# MAAI - AI Alert Interface

## 📌 Overview
MAAI (Medical AI Alert Interface) is a **real-time patient monitoring and alert system** combining **Arduino-based sensor acquisition** with a **Python-based PyQt5 desktop dashboard**.  
It is designed to monitor **vital signs** and **motion data** in elderly or high-risk patients, providing visualizations, real-time alerts, and video-based person detection.

---

## 🚀 Features
- **Real-time data acquisition** from sensors (Acceleration, Gyroscope, Temperature, ECG, BPM, Fall Detection)
- **HTTP-based communication** between Arduino/ESP32 and the desktop application
- **Live graphs** for:
  - Heart Rate (BPM)
  - Acceleration
  - Gyroscope
  - Temperature
- **Fall detection alert** with visual notifications
- **ECG connection status monitoring**
- **Live webcam feed** with AI-powered **person detection** (HOG + OpenCV)
- **Dark/Light mode toggle** for the dashboard
- **Patient information display** (Name, Age, Blood Type, Allergies, Photo)

---

## 🛠 Tech Stack
- **Frontend/Dashboard**: Python, PyQt5, PyQtGraph
- **Computer Vision**: OpenCV (HOG Person Detector)
- **Backend Communication**: HTTP Server (Python `http.server`)
- **Microcontroller**: Arduino/ESP32 (C++ with `.ino` file)
- **Data Transmission**: HTTP POST from microcontroller to Python GUI

---

## 📂 Project Structure
