#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);  // IST (GMT+5:30)

// Pin Definitions
#define LED_PIN 4
#define ONE_WIRE_BUS 19
#define ECG_PIN 36
#define LO_PLUS 34
#define LO_MINUS 35

// WiFi credentials
const char* ssid = "stormknight";
const char* password = "123445677";

// Server details
const char* serverUrl = "http://10.175.89.131:5000";

// Fall Detection Thresholds
#define FALL_THRESHOLD 8.0
#define IMPACT_THRESHOLD 10.0
#define GYRO_THRESHOLD 70.0
#define STILLNESS_GYRO_THRESHOLD 10.0
#define STILLNESS_TIME 3000

// Moving average filter parameters
#define FILTER_SIZE 10
float accelBuffer[FILTER_SIZE] = {0};
float gyroBuffer[FILTER_SIZE] = {0};
int filterIndex = 0;

// Data transmission buffer
#define BUFFER_SIZE 10
String dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastTransmissionTime = 0;
const unsigned long TRANSMISSION_INTERVAL = 1000; // Send data every second

// Task handles
TaskHandle_t fallDetectionTaskHandle = NULL;
TaskHandle_t temperatureTaskHandle = NULL;
TaskHandle_t sensorUpdateTaskHandle = NULL;
TaskHandle_t ecgTaskHandle = NULL;

// Mutex for sensor data protection
SemaphoreHandle_t sensorMutex;

// ECG Parameters
#define ECG_THRESHOLD 500  // Reduced from 600 for better peak detection
#define BPM_SAMPLE_SIZE 10
#define PEAK_DEBOUNCE_TIME 200 
// ECG Filtering
#define ECG_FILTER_SIZE 5
int ecgBuffer[ECG_FILTER_SIZE] = {0};
int ecgBufferIndex = 0;

// Initialize sensors
Adafruit_MPU6050 mpu;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Shared variables protected by mutex
struct SensorData {
    float filteredAcceleration;
    float filteredGyro;
    float temperature;
    int ecg;
    int bpm;
    unsigned long timestamp;
    int fall;
    int lead;
} sensorData;

// ECG peak intervals
unsigned long peakIntervals[BPM_SAMPLE_SIZE] = {0};
int peakIndex = 0;
unsigned long lastPeakTime = 0;
bool peakDetected = false;
unsigned long lastPeakDetectionTime = 0;

// Forward declarations
void initializeSensors();
float getTotalAcceleration();
float getTotalGyro();
float readTemperature();
void updateFilter();
bool detectFreeFall(float accel, float gyro);
bool detectImpact(float accel);
bool checkStillness(float gyro);
void calculateBPM();
void printSensorData();
int getFilteredECG(int newValue);

// **WiFi Reconnection**
void ensureWiFiConnected() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\nWiFi Reconnected.");
    }
}

// **Send Data to Server**
void sendDataToServer(String data) {
    ensureWiFiConnected();
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverUrl);
        http.addHeader("Content-Type", "text/plain");
        int httpResponseCode = http.POST(data);
        Serial.printf("HTTP Response code: %d\n", httpResponseCode);
        http.end();
    } else {
        Serial.println("WiFi Disconnected");
    }
}

// **Handle Data Buffering & Transmission**
void handleDataTransmission(String newData) {
    dataBuffer[bufferIndex] = newData;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    
    if (millis() - lastTransmissionTime >= TRANSMISSION_INTERVAL) {
        String allData;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            if (dataBuffer[i] != "") {
                allData += dataBuffer[i];
            }
        }
        
        if (allData.length() > 0) {
            sendDataToServer(allData);
        }
        
        memset(dataBuffer, 0, sizeof(dataBuffer)); // Clear buffer
        bufferIndex = 0;
        lastTransmissionTime = millis();
    }
}

// Function implementations
bool detectFreeFall(float accel, float gyro) {
    return (accel < FALL_THRESHOLD && gyro > GYRO_THRESHOLD);
}

bool detectImpact(float accel) {
    return (accel > IMPACT_THRESHOLD);
}

bool checkStillness(float gyro) {
    return (gyro <= STILLNESS_GYRO_THRESHOLD);
}

void fallDetectionTask(void *parameter) {
    bool possibleFall = false;
    bool impactDetected = false;
    unsigned long impactCheckStartTime = 0;
    const int IMPACT_WAIT_TIME = 1000;  // Time to wait for impact confirmation
    const int GYRO_CHECKS = 30;         // Number of consecutive stillness checks
    const int REQUIRED_STILL_COUNT = 16; // At least 6 out of 10 readings must be below threshold
    const int GYRO_CHECK_INTERVAL = 100; // Interval (ms) between gyro checks
    

    while (1) {
        float currentAccel, currentGyro;
        sensorData.fall = 0;

        // Read sensor data safely
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        currentAccel = sensorData.filteredAcceleration;
        currentGyro = sensorData.filteredGyro;
        xSemaphoreGive(sensorMutex);

        // Step 1: Detect Free Fall
        if (!possibleFall && !impactDetected && detectFreeFall(currentAccel, currentGyro)) {
            Serial.println("Possible Free Fall Detected! Pausing to confirm impact...");
            digitalWrite(LED_PIN, HIGH);
            possibleFall = true;
            impactCheckStartTime = millis();
        }

        // Step 2: Wait for Impact
        if (possibleFall && !impactDetected) {
            if (detectImpact(currentAccel)) {
                Serial.println("Impact Detected! Checking for stillness...");
                impactDetected = true;

                // Step 3: Check for Stillness (at least 6 out of 10 readings must be below threshold)
                int stillCount = 0;

                for (int i = 0; i < GYRO_CHECKS; i++) {
                    // Read gyro data safely
                    xSemaphoreTake(sensorMutex, portMAX_DELAY);
                    currentGyro = sensorData.filteredGyro;
                    xSemaphoreGive(sensorMutex);

                    Serial.printf("Gyro Check %d: %.2f\n", i + 1, currentGyro);

                    if (currentGyro <= STILLNESS_GYRO_THRESHOLD) {
                        stillCount++; // Count the number of times stillness is detected
                    }

                    vTaskDelay(pdMS_TO_TICKS(GYRO_CHECK_INTERVAL)); // Delay for next check
                }

                // Step 4: Confirm Fall if at least 6 out of 10 readings were still
                if (stillCount >= REQUIRED_STILL_COUNT) {
                    Serial.println("FALL CONFIRMED! Person may need help.");
                    digitalWrite(LED_PIN, HIGH);
                    digitalWrite(15, HIGH);
                    sensorData.fall = 1;
                    vTaskDelay(pdMS_TO_TICKS(3000)); // LED stays on to indicate emergency
                } else {
                    Serial.println("Movement detected - False alarm. Resetting.");
                    sensorData.fall = 0;
                }

                possibleFall = false;
                impactDetected = false;
                digitalWrite(LED_PIN, LOW);
                digitalWrite(15, LOW);
            } 
            else if (millis() - impactCheckStartTime >= IMPACT_WAIT_TIME) {
                // If no impact detected within time limit, reset
                Serial.println("No impact detected. Resuming normal operation.");
                possibleFall = false;
                digitalWrite(LED_PIN, LOW);
                digitalWrite(15, LOW);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// **Initialize Sensors**
void initializeSensors() {
    Wire.begin();
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensors.begin();
}

float getTotalAcceleration() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
}

float getTotalGyro() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z) * 180 / M_PI;
}

void updateFilter() {
    accelBuffer[filterIndex] = getTotalAcceleration();
    gyroBuffer[filterIndex] = getTotalGyro();
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
}

float readTemperature() {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    
    if (tempC != DEVICE_DISCONNECTED_C) {
        return tempC;
    } else {
        Serial.println("Error: Could not read temperature data");
        return -1;
    }
}

void sensorUpdateTask(void *parameter) {
    while (1) {
        updateFilter();
        float newAccel = getTotalAcceleration();
        float newGyro = getTotalGyro();
        
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        sensorData.filteredAcceleration = newAccel;
        sensorData.filteredGyro = newGyro;
        xSemaphoreGive(sensorMutex);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

String getCurrentTimestamp() {
    timeClient.update();
    return timeClient.getFormattedTime();
}

// **Temperature Task**
void temperatureTask(void *parameter) {
    while (1) {
        sensors.requestTemperatures();
        float temp = sensors.getTempCByIndex(0);
        String timestamp = getCurrentTimestamp(); 
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        sensorData.temperature = temp + 3;
        sensorData.timestamp = millis();
        String formattedData = "$" + timestamp + "," + String(sensorData.filteredAcceleration, 2) + "," + String(sensorData.filteredGyro, 2) + "," + String(sensorData.temperature, 2) + "," + String(sensorData.ecg) + "," + String(sensorData.bpm) + ","+String(sensorData.fall) + ","+String(sensorData.lead)+"#\r\n";
        Serial.print(formattedData);
        xSemaphoreGive(sensorMutex);
        handleDataTransmission(formattedData);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Modified calculateBPM function to be more robust
void calculateBPM() {
    unsigned long sumIntervals = 0;
    int validIntervals = 0;
    
    // More permissive at the start, then get stricter
    unsigned long minInterval = (millis() < 30000) ? 250 : 300;
    unsigned long maxInterval = (millis() < 30000) ? 2400 : 2000;
    int minRequiredIntervals = (millis() < 30000) ? 1 : 2;
    
    for (int i = 0; i < BPM_SAMPLE_SIZE; i++) {
        // Filter out unreasonable intervals (too short or too long)
        if (peakIntervals[i] > minInterval && peakIntervals[i] <maxInterval) {
            sumIntervals += peakIntervals[i];
            validIntervals++;
        }
    }

    if (validIntervals >= minRequiredIntervals) {
        unsigned long averageInterval = sumIntervals / validIntervals;
        
        // Prevent division by zero
        if (averageInterval > 0) {
            int newBpm = 60000 / averageInterval;
            
            // Check if BPM is in a reasonable range
            if (newBpm >= 40 && newBpm <= 200) {
                // xSemaphoreTake(sensorMutex, portMAX_DELAY);
                sensorData.bpm = newBpm;
                // Serial.printf("New BPM calculated: %d (from %d intervals)\n", newBpm, validIntervals);
                // xSemaphoreGive(sensorMutex);
            }
        }
    }
    
    // Debug output to help diagnose issues
    // Serial.printf("BPM calculation: Found %d valid intervals out of %d\n", 
    //               validIntervals, BPM_SAMPLE_SIZE);
}

int getFilteredECG(int newValue) {
    ecgBuffer[ecgBufferIndex] = newValue;
    ecgBufferIndex = (ecgBufferIndex + 1) % ECG_FILTER_SIZE;
    
    int sum = 0;
    for (int i = 0; i < ECG_FILTER_SIZE; i++) {
        sum += ecgBuffer[i];
    }
    return sum / ECG_FILTER_SIZE;
}

// Improved ECG Task with better adaptivity
void ecgTask(void *parameter) {
    unsigned long lastBpmCalcTime = 0;
    unsigned long lastReliableReading = 0;
    int minEcgValue = 4095;
    int maxEcgValue = 0;
    unsigned long calibrationEnd = millis() + 5000; // First 5 seconds for calibration
    
    // Initialize peak intervals
    for (int i = 0; i < BPM_SAMPLE_SIZE; i++) {
        peakIntervals[i] = 0;
    }
    
    while (1) {
        int rawEcgValue = analogRead(ECG_PIN);
        int loPlus = digitalRead(LO_PLUS);
        int loMinus = digitalRead(LO_MINUS);
        unsigned long currentTime = millis();
        sensorData.lead = 0;
        
        // Dynamic adaptation for signal range
        if (rawEcgValue <minEcgValue) 
            minEcgValue = rawEcgValue;
        else if (currentTime > calibrationEnd)
            minEcgValue += (rawEcgValue - minEcgValue) * 0.001; // Very slow increase
            
        if (rawEcgValue > maxEcgValue)
            maxEcgValue = rawEcgValue;
        else if (currentTime > calibrationEnd)
            maxEcgValue -= (maxEcgValue - rawEcgValue) * 0.001; // Very slow decrease
        
        if (loPlus == HIGH || loMinus == HIGH) {
            // Leads are not properly connected
            if (currentTime - lastReliableReading > 3000) {
              sensorData.lead = 0;
                // Serial.println("ECG leads disconnected for over 3 seconds");
            }
        } else {
            // Process ECG when leads are connected
            sensorData.lead = 1;
            lastReliableReading = currentTime;
            int ecgValue = getFilteredECG(rawEcgValue);

            // Store ECG value even if not a peak
            xSemaphoreTake(sensorMutex, portMAX_DELAY);
            sensorData.ecg = ecgValue > 0 ? ecgValue : sensorData.ecg;
            xSemaphoreGive(sensorMutex);

            // Check for poor signal quality
            if (maxEcgValue - minEcgValue <200 && currentTime > calibrationEnd) {
                // Serial.println("Warning: Low ECG signal variation - check sensor placement");
            }

            // Detect a peak based on threshold with dynamic adjustment
            int dynamicThreshold = minEcgValue + (maxEcgValue - minEcgValue) * 0.7;
            int actualThreshold = min(ECG_THRESHOLD, dynamicThreshold);
            
            if (ecgValue > actualThreshold && (currentTime - lastPeakTime) > PEAK_DEBOUNCE_TIME) {
                unsigned long interval = currentTime - lastPeakTime;
                // More permissive at start
                unsigned long minValidInterval = (currentTime < 30000) ? 250 : 300;
                unsigned long maxValidInterval = (currentTime < 30000) ? 2400 : 2000;
                
                if (interval > minValidInterval && interval < maxValidInterval) {
                    peakIntervals[peakIndex] = interval;
                    peakIndex = (peakIndex + 1) % BPM_SAMPLE_SIZE;
                    // Serial.printf("Peak detected! Interval: %lu ms\n", interval);
                }
                lastPeakTime = currentTime;
                peakDetected = true;
            } else {
                peakDetected = false;
            }

            // Reset if no peaks detected for too long
            if (currentTime - lastPeakTime > 5000 && currentTime > calibrationEnd) {
                minEcgValue = min(minEcgValue + 100, 2047);
                maxEcgValue = max(maxEcgValue - 100, 2048);
                // Serial.println("No peaks detected for 5 seconds - adjusting thresholds");
            }

            // Calculate BPM every 2 seconds
            if (currentTime - lastBpmCalcTime > 2000) {
                calculateBPM();
                lastBpmCalcTime = currentTime;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(LO_PLUS, INPUT);
    pinMode(LO_MINUS, INPUT);
    pinMode(ECG_PIN, INPUT);
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    
    // **Connect WiFi Initially**
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH);
    // Start NTP Client
    timeClient.begin();
    timeClient.update();
    
    sensorMutex = xSemaphoreCreateMutex();
    initializeSensors();
    
    // Initialize sensor data with default values
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    sensorData.bpm = 0;
    sensorData.ecg = 0;
    xSemaphoreGive(sensorMutex);
    
    xTaskCreatePinnedToCore(
        fallDetectionTask,
        "FallDetection",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        &fallDetectionTaskHandle,
        1
    );

    xTaskCreatePinnedToCore(
        sensorUpdateTask,
        "SensorUpdate",
        2048,
        NULL,
        configMAX_PRIORITIES - 2,
        &sensorUpdateTaskHandle,
        1
    );

    xTaskCreatePinnedToCore(temperatureTask, "Temperature", 8192, NULL, 1, &temperatureTaskHandle, 0);
    xTaskCreatePinnedToCore(ecgTask, "ECG", 4096, NULL, 1, &ecgTaskHandle, 0);
}

void loop() {
    ensureWiFiConnected();  // Keep checking WiFi connection
    vTaskDelay(pdMS_TO_TICKS(5000));  // Give room for tasks to run
}