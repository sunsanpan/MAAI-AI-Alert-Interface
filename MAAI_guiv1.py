import sys
import threading
import time
from datetime import datetime
from collections import deque
import numpy as np
import cv2
import imutils
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout,
                             QHBoxLayout, QWidget, QComboBox, QFrame, QMessageBox,
                             QPushButton, QSizePolicy, QSplitter)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont, QPalette, QColor, QImage, QPixmap, QPainter, QPainterPath

import pyqtgraph as pg
from http.server import HTTPServer, BaseHTTPRequestHandler

# A new QObject to emit video frames from the worker thread


class VideoStream(QObject):
    frame_ready = pyqtSignal(np.ndarray)


class VideoThread(QThread):
    def __init__(self, video_stream):
        super().__init__()
        self.video_stream = video_stream
        self.running = False
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            self.running = False
            return

        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = imutils.resize(frame, width=min(640, frame.shape[1]))
            (boxes, weights) = self.hog.detectMultiScale(frame,
                                                         winStride=(4, 4),
                                                         padding=(8, 8),
                                                         scale=1.05)

            for (x, y, w, h) in boxes:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "Person", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.video_stream.frame_ready.emit(frame)

        cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False
        self.wait()


class DataHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length).decode('utf-8')
        self.server.app.process_data(post_data)
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(b'OK')

    def log_message(self, format, *args):
        pass


class HttpDataReceiver(QObject):
    data_received = pyqtSignal(str)

    def __init__(self, host='0.0.0.0', port=5000):
        super().__init__()
        self.host = host
        self.port = port
        self.running = False
        self.server = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run_server)
        self.thread.daemon = True
        self.thread.start()

    def run_server(self):
        server_class = HTTPServer
        httpd = server_class((self.host, self.port), DataHandler)
        httpd.app = self
        print(f"HTTP Server started on {self.host}:{self.port}")

        while self.running:
            try:
                httpd.handle_request()
            except Exception as e:
                print(f"HTTP Server error: {e}")
                break

        httpd.server_close()
        print("HTTP Server stopped")

    def process_data(self, data):
        self.data_received.emit(data)

    def stop(self):
        self.running = False
        try:
            import urllib.request
            urllib.request.urlopen(f"http://{self.host}:{self.port}").close()
        except:
            pass


class CareTracApp(QMainWindow):
    def __init__(self):
        super().__init__()

        # Patient Info (Easily changeable)
        self.patient_info = {
            'name': 'Ramesh Kumar',
            'age': '72',
            'blood_type': 'O+',
            'allergies': 'Penicillin',
            'image_path': 'patient_image.png'  # Make sure this file exists
        }

        # Data storage
        self.timestamp = None
        self.acceleration = 0
        self.gyroscope = 0
        self.temperature = 0
        self.bpm = 0
        self.fall_status = 0
        self.ecg_connection = 0

        # Graph data storage
        self.bpm_data = deque(maxlen=200)
        self.accel_data = deque(maxlen=100)
        self.gyro_data = deque(maxlen=100)
        self.temp_data = deque(maxlen=100)
        self.timestamps = deque(maxlen=200)

        # Theme management
        self.is_dark_mode = False

        self.init_ui()
        self.apply_theme()

        self.data_receiver = HttpDataReceiver(host='0.0.0.0', port=5000)
        self.data_receiver.data_received.connect(self.process_data)
        self.data_receiver.start()

        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.update_ui)
        self.refresh_timer.start(100)

        self.fall_alert_timer = QTimer()
        self.fall_alert_timer.timeout.connect(self.hide_fall_alert)
        self.fall_alert_timer.setSingleShot(True)

        self.video_stream = VideoStream()
        self.video_stream.frame_ready.connect(self.update_video_frame)
        self.video_thread = VideoThread(self.video_stream)
        self.video_thread.start()

    def init_ui(self):
        self.setWindowTitle('MAAI : AI Alert Interface')
        self.setGeometry(100, 100, 1400, 900)

        main_widget = QWidget()
        self.main_layout = QHBoxLayout()
        main_widget.setLayout(self.main_layout)
        self.setCentralWidget(main_widget)

        # QSplitter for resizable panels
        self.splitter = QSplitter(Qt.Horizontal)
        self.main_layout.addWidget(self.splitter)

        # Left panel for health monitoring
        self.health_panel = QWidget()
        health_layout = QVBoxLayout()
        health_layout.setContentsMargins(0, 0, 0, 0)
        self.health_panel.setLayout(health_layout)

        # Header
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(10, 10, 10, 10)
        title_label = QLabel("MAAI : AI Alert Interface")
        title_font = QFont("Arial", 24, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setObjectName("titleLabel")
        title_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(title_label)

        self.theme_button = QPushButton("🌙 Dark Mode")
        self.theme_button.clicked.connect(self.toggle_theme)
        header_layout.addWidget(self.theme_button)
        health_layout.addLayout(header_layout)

        # Vitals display
        vitals_frame = QFrame()
        vitals_frame.setObjectName("vitalsFrame")
        vitals_frame.setContentsMargins(20, 10, 20, 10)
        vitals_layout = QHBoxLayout()
        vitals_frame.setLayout(vitals_layout)

        temp_layout = QVBoxLayout()
        temp_label = QLabel("Temperature")
        temp_label.setAlignment(Qt.AlignCenter)
        temp_label.setObjectName("vitalsLabel")
        self.temp_value = QLabel("--°C")
        self.temp_value.setFont(QFont("Arial", 20))
        self.temp_value.setAlignment(Qt.AlignCenter)
        self.temp_value.setObjectName("vitalsValue")
        temp_layout.addWidget(temp_label)
        temp_layout.addWidget(self.temp_value)

        bpm_layout = QVBoxLayout()
        bpm_label = QLabel("Heart Rate")
        bpm_label.setAlignment(Qt.AlignCenter)
        bpm_label.setObjectName("vitalsLabel")
        self.bpm_value = QLabel("-- BPM")
        self.bpm_value.setFont(QFont("Arial", 20))
        self.bpm_value.setAlignment(Qt.AlignCenter)
        self.bpm_value.setObjectName("vitalsValue")
        bpm_layout.addWidget(bpm_label)
        bpm_layout.addWidget(self.bpm_value)

        # ECG Connection status
        ecg_status_layout = QVBoxLayout()
        ecg_status_label = QLabel("ECG Status")
        ecg_status_label.setAlignment(Qt.AlignCenter)
        ecg_status_label.setStyleSheet("font-weight: bold; color: #3498db;")
        self.ecg_status_value = QLabel("Disconnected")
        self.ecg_status_value.setFont(QFont("Arial", 16))
        self.ecg_status_value.setAlignment(Qt.AlignCenter)
        self.ecg_status_value.setStyleSheet("color: #e74c3c;")
        ecg_status_layout.addWidget(ecg_status_label)
        ecg_status_layout.addWidget(self.ecg_status_value)

        vitals_layout.addLayout(temp_layout)
        vitals_layout.addLayout(bpm_layout)
        vitals_layout.addLayout(ecg_status_layout)
        health_layout.addWidget(vitals_frame)

        # BPM Graph
        self.bpm_plot_widget = self.create_minimal_graph_widget(
            "Heart Rate (BPM) Trend", 'BPM', (231, 76, 60))
        self.bpm_plot = self.bpm_plot_widget.findChild(pg.PlotWidget)
        self.bpm_plot.setYRange(50, 120)
        self.bpm_curve = self.bpm_plot.plot(
            pen=pg.mkPen(color=(231, 76, 60), width=2))
        health_layout.addWidget(self.bpm_plot_widget, stretch=2)

        # Inline motion and temperature graphs
        mini_graphs_layout = QHBoxLayout()
        mini_graphs_layout.setSpacing(10)

        # Acceleration graph
        self.accel_plot_widget = self.create_minimal_graph_widget(
            "Acceleration", 'm/s²', (52, 152, 219))
        self.accel_plot = self.accel_plot_widget.findChild(pg.PlotWidget)
        self.accel_plot.setYRange(0, 20)  # acceleration range
        self.accel_curve = self.accel_plot.plot(
            pen=pg.mkPen(color=(52, 152, 219), width=1))
        mini_graphs_layout.addWidget(self.accel_plot_widget, stretch=1)

        # Gyroscope graph
        self.gyro_plot_widget = self.create_minimal_graph_widget(
            "Gyroscope", '°/s', (46, 204, 113))
        self.gyro_plot = self.gyro_plot_widget.findChild(pg.PlotWidget)
        self.gyro_plot.setYRange(0, 10)
        self.gyro_curve = self.gyro_plot.plot(
            pen=pg.mkPen(color=(46, 204, 113), width=1))
        mini_graphs_layout.addWidget(self.gyro_plot_widget, stretch=1)

        # Temperature graph
        self.temp_plot_widget = self.create_minimal_graph_widget(
            "Temperature", '°C', (241, 196, 15))
        self.temp_plot = self.temp_plot_widget.findChild(pg.PlotWidget)
        self.temp_plot.setYRange(0, 40)
        self.temp_curve = self.temp_plot.plot(
            pen=pg.mkPen(color=(241, 196, 15), width=1))
        mini_graphs_layout.addWidget(self.temp_plot_widget, stretch=1)

        health_layout.addLayout(mini_graphs_layout, stretch=1)

        # Motion data dropdown section
        motion_layout = QHBoxLayout()
        motion_layout.setContentsMargins(10, 0, 10, 0)
        motion_label = QLabel("Motion Data:")
        motion_label.setObjectName("motionLabel")
        self.motion_combo = QComboBox()
        self.motion_combo.addItem("Select motion data...")
        self.motion_combo.addItem("Acceleration")
        self.motion_combo.addItem("Gyroscope")
        self.motion_combo.currentIndexChanged.connect(
            self.update_motion_display)
        self.motion_combo.setObjectName("motionCombo")
        motion_layout.addWidget(motion_label)
        motion_layout.addWidget(self.motion_combo)
        health_layout.addLayout(motion_layout)

        self.motion_value_label = QLabel()
        self.motion_value_label.setAlignment(Qt.AlignCenter)
        self.motion_value_label.setObjectName("motionValueLabel")
        health_layout.addWidget(self.motion_value_label)

        # Fall alert frame
        self.fall_alert_frame = QFrame()
        self.fall_alert_frame.setObjectName("fallAlertFrame")
        fall_alert_layout = QVBoxLayout()
        self.fall_alert_label = QLabel("FALL DETECTED\nPATIENT MAY NEED HELP!")
        self.fall_alert_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.fall_alert_label.setObjectName("fallAlertLabel")
        self.fall_alert_label.setAlignment(Qt.AlignCenter)
        fall_alert_layout.addWidget(self.fall_alert_label)
        self.fall_alert_frame.setLayout(fall_alert_layout)
        self.fall_alert_frame.setVisible(False)
        health_layout.addWidget(self.fall_alert_frame)

        # Right panel for video and patient info
        self.video_and_info_panel = QWidget()
        video_and_info_layout = QVBoxLayout()
        video_and_info_layout.setContentsMargins(10, 10, 10, 10)

        # Patient Info Frame (always visible)
        self.patient_info_frame = QFrame()
        self.patient_info_frame.setObjectName("patientInfoFrame")
        self.patient_info_frame.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.patient_info_frame.setMinimumHeight(250)
        patient_info_layout = QVBoxLayout()
        patient_info_layout.setContentsMargins(10, 10, 10, 10)

        # Patient Image
        self.patient_image_label = QLabel()
        self.patient_image_label.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.patient_image_label.setFixedSize(128, 128)
        self.patient_image_label.setAlignment(Qt.AlignCenter)
        self.patient_image_label.setPixmap(self.get_scaled_pixmap(
            self.patient_info['image_path'], 128, 128))

        patient_info_layout.addWidget(
            self.patient_image_label, alignment=Qt.AlignCenter)

        # Patient Details
        self.patient_name_label = QLabel(f"Name: {self.patient_info['name']}")
        self.patient_name_label.setObjectName("patientDetailLabel")
        self.patient_name_label.setAlignment(Qt.AlignCenter)
        self.patient_age_label = QLabel(f"Age: {self.patient_info['age']}")
        self.patient_age_label.setObjectName("patientDetailLabel")
        self.patient_age_label.setAlignment(Qt.AlignCenter)
        self.patient_blood_label = QLabel(
            f"Blood Type: {self.patient_info['blood_type']}")
        self.patient_blood_label.setObjectName("patientDetailLabel")
        self.patient_blood_label.setAlignment(Qt.AlignCenter)
        self.patient_allergies_label = QLabel(
            f"Allergies: {self.patient_info['allergies']}")
        self.patient_allergies_label.setObjectName("patientDetailLabel")
        self.patient_allergies_label.setAlignment(Qt.AlignCenter)

        patient_info_layout.addWidget(self.patient_name_label)
        patient_info_layout.addWidget(self.patient_age_label)
        patient_info_layout.addWidget(self.patient_blood_label)
        patient_info_layout.addWidget(self.patient_allergies_label)
        patient_info_layout.addStretch(1)

        self.patient_info_frame.setLayout(patient_info_layout)
        video_and_info_layout.addWidget(self.patient_info_frame)

        # Video Frame
        video_layout = QVBoxLayout()
        video_layout.setContentsMargins(0, 0, 0, 0)

        video_title_label = QLabel("Live Video & Person Detection")
        video_title_label.setAlignment(Qt.AlignCenter)
        video_title_label.setObjectName("videoTitleLabel")
        video_title_label.setFont(QFont("Arial", 16, QFont.Bold))
        video_layout.addWidget(video_title_label)

        self.video_label = QLabel()
        self.video_label.setScaledContents(True)
        self.video_label.setAlignment(Qt.AlignCenter)
        video_layout.addWidget(self.video_label)

        video_and_info_layout.addLayout(video_layout, stretch=1)

        self.video_and_info_panel.setLayout(video_and_info_layout)

        # Add panels to the splitter
        self.splitter.addWidget(self.health_panel)
        self.splitter.addWidget(self.video_and_info_panel)
        # Initial sizes for left and right panels
        self.splitter.setSizes([600, 800])

        self.statusBar().setObjectName("statusBar")
        self.statusBar().showMessage("Ready. Waiting for data...")

    def create_minimal_plot(self):
        plot = pg.PlotWidget()
        plot.showGrid(x=False, y=True, alpha=0.5)
        plot.hideAxis('bottom')
        plot.hideButtons()
        plot.setMouseEnabled(x=False, y=False)
        plot.addLegend()
        plot.getPlotItem().layout.setContentsMargins(0, 0, 0, 0)
        return plot

    def create_minimal_graph_widget(self, title, y_label, color):
        widget = QFrame()
        widget.setObjectName("graphFrame")
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setObjectName("graphTitleLabel")
        layout.addWidget(title_label)

        plot = self.create_minimal_plot()
        plot.setLabel('left', y_label)

        layout.addWidget(plot, stretch=1)
        widget.setLayout(layout)
        return widget

    def get_scaled_pixmap(self, image_path, width, height):
        try:
            image = QImage(image_path)
            if image.isNull():
                print(
                    f"Warning: Image file not found at {image_path}. Using a placeholder.")
                image = QImage(width, height, QImage.Format_RGB32)
                image.fill(QColor("gray"))

            pixmap = QPixmap.fromImage(image)
            return pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        except Exception as e:
            print(f"Error loading pixmap: {e}")
            return QPixmap(width, height)

    def apply_theme(self):
        if self.is_dark_mode:
            self.setStyleSheet("""
                QMainWindow, QWidget { background-color: #1a1a2e; color: #e94560; }
                QFrame#vitalsFrame { background-color: #2b2e4a; border-radius: 12px; border: 1px solid #5a5a7b; }
                QLabel#titleLabel { color: #53d2dc; }
                QLabel#vitalsLabel { color: #c4d7e0; font-weight: bold; }
                QLabel#vitalsValue { color: #e94560; }
                QLabel#graphTitleLabel, QLabel#motionLabel { color: #c4d7e0; font-weight: bold; }
                QFrame#graphFrame { background-color: #2b2e4a; border-radius: 12px; }
                QComboBox { background-color: #53d2dc; color: #1a1a2e; padding: 5px; border-radius: 6px; }
                QPushButton { background-color: #53d2dc; color: #1a1a2e; padding: 5px; border-radius: 6px; }
                QLabel#motionValueLabel { color: #f9c21b; font-family: monospace; }
                QFrame#fallAlertFrame { background-color: #f03e3e; border-radius: 12px; border: 2px solid #a82e2e; }
                QLabel#fallAlertLabel { color: white; font-weight: bold; }
                QStatusBar { background-color: #2b2e4a; color: #c4d7e0; }
                QFrame#patientInfoFrame { background-color: #2b2e4a; border-radius: 12px; }
                QLabel#patientDetailLabel { color: #c4d7e0; font-weight: bold; }
                
                QLabel#videoTitleLabel {
                    border: 1px solid #5a5a7b;
                    background-color: #2b2e4a;
                    border-radius: 6px;
                    padding: 5px;
                    margin-bottom: 5px;
                }
                QSplitter::handle {
                    background-color: #5a5a7b;
                    border: 1px solid #1a1a2e;
                }
            """)
            plot_background = '#2b2e4a'
            self.bpm_plot.setBackground(plot_background)
            self.bpm_plot.getAxis('left').setTextPen('white')

            self.accel_plot.setBackground(plot_background)
            self.accel_plot.getAxis('left').setTextPen('white')
            self.gyro_plot.setBackground(plot_background)
            self.gyro_plot.getAxis('left').setTextPen('white')
            self.temp_plot.setBackground(plot_background)
            self.temp_plot.getAxis('left').setTextPen('white')
            self.theme_button.setText("☀️ Light Mode")
        else:
            self.setStyleSheet("""
                QMainWindow, QWidget { background-color: #f0f4f8; color: #2c3e50; }
                QFrame#vitalsFrame { background-color: white; border-radius: 12px; border: 1px solid #e0e0e0; }
                QLabel#titleLabel { color: #3498db; }
                QLabel#vitalsLabel { color: #2c3e50; font-weight: bold; }
                QLabel#vitalsValue { color: #e74c3c; }
                QLabel#graphTitleLabel, QLabel#motionLabel { color: #2980b9; font-weight: bold; }
                QFrame#graphFrame { background-color: white; border-radius: 12px; }
                QComboBox { background-color: #3498db; color: white; padding: 5px; border-radius: 6px; }
                QPushButton { background-color: #3498db; color: white; padding: 5px; border-radius: 6px; }
                QLabel#motionValueLabel { color: #e67e22; font-family: monospace; }
                QFrame#fallAlertFrame { background-color: #e74c3c; border-radius: 12px; border: 2px solid #c0392b; }
                QLabel#fallAlertLabel { color: black; font-weight: bold; }
                QStatusBar { background-color: #e0e0e0; color: #2c3e50; }
                QFrame#patientInfoFrame { background-color: white; border-radius: 12px; }
                QLabel#patientDetailLabel { color: #2c3e50; font-weight: bold; }

                QLabel#videoTitleLabel {
                    border: 1px solid #e0e0e0;
                    background-color: white;
                    border-radius: 6px;
                    padding: 5px;
                    margin-bottom: 5px;
                }
                QSplitter::handle {
                    background-color: #e0e0e0;
                    border: 1px solid #f0f4f8;
                }
            """)
            plot_background = 'white'
            self.bpm_plot.setBackground(plot_background)
            self.bpm_plot.getAxis('left').setTextPen('black')

            self.accel_plot.setBackground(plot_background)
            self.accel_plot.getAxis('left').setTextPen('black')
            self.gyro_plot.setBackground(plot_background)
            self.gyro_plot.getAxis('left').setTextPen('black')
            self.temp_plot.setBackground(plot_background)
            self.temp_plot.getAxis('left').setTextPen('black')
            self.theme_button.setText("🌙 Dark Mode")

    def toggle_theme(self):
        self.is_dark_mode = not self.is_dark_mode
        self.apply_theme()

    def process_data(self, data_packet):
        try:
            data_str = data_packet.strip()
            if data_str.startswith('$') and '#' in data_str:
                data_str = data_str[1:data_str.find('#')]
                parts = data_str.split(',')

                if len(parts) >= 8:
                    self.timestamp = parts[0].strip()
                    try:
                        self.acceleration = float(parts[1].strip())
                    except ValueError:
                        print(
                            f"Failed to convert acceleration value: '{parts[1].strip()}'")
                        self.acceleration = 0.0

                    try:
                        self.gyroscope = float(parts[2].strip())
                    except ValueError:
                        print(
                            f"Failed to convert gyroscope value: '{parts[2].strip()}'")
                        self.gyroscope = 0.0

                    try:
                        self.temperature = float(parts[3].strip())
                    except ValueError:
                        print(
                            f"Failed to convert temperature value: '{parts[3].strip()}'")
                        self.temperature = 0.0

                    try:
                        self.bpm = int(parts[5].strip())
                    except ValueError:
                        print(
                            f"Failed to convert BPM value: '{parts[5].strip()}'")
                        self.bpm = 0

                    try:
                        self.fall_status = int(parts[6].strip())
                    except ValueError:
                        print(
                            f"Failed to convert fall status value: '{parts[6].strip()}'")
                        self.fall_status = 0

                    try:
                        self.ecg_connection = int(parts[7].strip())
                    except ValueError:
                        print(
                            f"Failed to convert ECG connection value: '{parts[7].strip()}'")
                        self.ecg_connection = 0

                    current_time = datetime.now().timestamp()
                    self.timestamps.append(current_time)
                    self.bpm_data.append(self.bpm)
                    self.accel_data.append(self.acceleration)
                    self.gyro_data.append(self.gyroscope)
                    self.temp_data.append(self.temperature)

                    if self.fall_status == 1:
                        self.show_fall_alert()

                    self.statusBar().showMessage(
                        f"Data received at {datetime.now().strftime('%H:%M:%S')}")

        except Exception as e:
            print(f"Error processing data packet: {e}")

    def update_ui(self):
        self.temp_value.setText(f"{self.temperature:.1f}°C")
        self.bpm_value.setText(f"{self.bpm} BPM")

        if self.ecg_connection == 1:
            self.ecg_status_value.setText("Connected")
            self.ecg_status_value.setStyleSheet("color: #27ae60;")
        else:
            self.ecg_status_value.setText("Disconnected")
            self.ecg_status_value.setStyleSheet("color: #e74c3c;")

        if len(self.timestamps) > 0:
            base_time = self.timestamps[0]
            rel_times = [t - base_time for t in self.timestamps]

            # BPM (timestamps and bpm_data have same maxlen 200)
            x_bpm = rel_times[-len(self.bpm_data):]
            self.bpm_curve.setData(x=x_bpm, y=list(self.bpm_data))

            # Accel / Gyro / Temp: make x match each series length
            if len(self.accel_data) > 0:
                x_accel = rel_times[-len(self.accel_data):]
                self.accel_curve.setData(x=x_accel, y=list(self.accel_data))
            if len(self.gyro_data) > 0:
                x_gyro = rel_times[-len(self.gyro_data):]
                self.gyro_curve.setData(x=x_gyro, y=list(self.gyro_data))
            if len(self.temp_data) > 0:
                x_temp = rel_times[-len(self.temp_data):]
                self.temp_curve.setData(x=x_temp, y=list(self.temp_data))

            # Optional: autoscale small plots if values fall outside preset ranges
            # (uncomment to enable autoscaling)
            # try:
            #     self.accel_plot.enableAutoRange(True, True)
            #     self.temp_plot.enableAutoRange(True, True)
            # except Exception:
            #     pass

        self.update_motion_display()

    def update_video_frame(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        q_image = QImage(rgb_image.data, w, h,
                         bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap)

    def update_motion_display(self):
        selected = self.motion_combo.currentText()
        if selected == "Acceleration":
            text = f"Acceleration: {self.acceleration:.2f} m/s²"
            self.motion_value_label.setText(text)
        elif selected == "Gyroscope":
            text = f"Gyroscope: {self.gyroscope:.2f}°/s"
            self.motion_value_label.setText(text)
        else:
            self.motion_value_label.setText("")

    def show_fall_alert(self):
        self.fall_alert_frame.setVisible(True)
        self.fall_alert_timer.start(3000)

    def hide_fall_alert(self):
        self.fall_alert_frame.setVisible(False)

    def closeEvent(self, event):
        if hasattr(self, 'data_receiver'):
            self.data_receiver.stop()
        if hasattr(self, 'refresh_timer'):
            self.refresh_timer.stop()
        if hasattr(self, 'video_thread'):
            self.video_thread.stop()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    caretrac = CareTracApp()
    caretrac.show()
    sys.exit(app.exec_())
