import sys
import cv2
import math
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QFrame, QGridLayout, QComboBox, QTextEdit, QMessageBox
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QTextCursor, QBrush, QPolygon, QColor
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPoint
from serial.tools import list_ports
import serial
import traceback

class SerialReaderThread(QThread):
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port, baudrate=115200, timeout=0.5):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._is_running = True
        self.serial_connection = None

    def run(self):
        """Thread entry point. Opens the serial port and continuously reads data."""
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.serial_connection.flushInput()
        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to open serial port {self.port}: {e}")
            return
        except Exception as e:
            # Catch other unexpected errors when trying to open port
            self.error_occurred.emit(f"Unexpected error opening port {self.port}: {e}\n{traceback.format_exc()}")
            return
        
        while self._is_running:
            try:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        self.data_received.emit(data)
            except serial.SerialException as e:
                self.error_occurred.emit(f"Serial error: {e}")
                break
            except Exception as e:
                # Catch any other unexpected exceptions
                self.error_occurred.emit(
                    "Serial thread unexpected error: "
                    f"{e}\n{traceback.format_exc()}"
                )
                break

        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

    def stop(self):
        """Stops the thread gracefully."""
        self._is_running = False
        self.wait()

class RoverControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # -------------------------
        # 1) MAP Pixmap Setup
        # -------------------------
        self.map_size = (650, 300)
        self.map_pixmap = QPixmap(*self.map_size)
        self.map_pixmap.fill(Qt.white)

        # Keep track of dynamic lat/lon boundaries (for auto-scaling)
        self.min_lat = float('inf')
        self.max_lat = float('-inf')
        self.min_lon = float('inf')
        self.max_lon = float('-inf')

        # Operating mode (Manual / Auto)
        self.manual_mode = True

        # Keep track of the last GPS coordinate
        self.last_gps_lat = 0.0
        self.last_gps_lon = 0.0
        
        # GPS filtering parameters
        self.GPS_THRESHOLD = 0.00001  # Approximately 1 meter at equator
        self.GPS_BUFFER_SIZE = 5  # Number of readings to average
        self.gps_buffer = []  # Buffer for GPS readings averaging
        
        # Store all rover GPS points: list of tuples (lat, lon)
        self.map_gps_data = []
        # Store obstacle data: list of tuples (lat, lon, distance_mm)
        self.map_obstacle_data = []

        self.init_ui()

        # -------------------------
        # 2) OpenCV Video Capture
        # -------------------------
        # Attempt to open the first available camera
        self.cap = None
        for index in range(0, 5):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                self.cap = cap
                self.camera_status.setText(f"Camera: Connected (Index {index})")
                self.camera_status.setStyleSheet("font-size: 15px; color: green; font-weight: bold;")
                self.log_message(f"Camera connected on index: {index}")
                break
        if self.cap is None:
            self.camera_status.setText("Camera: Not Connected")
            self.camera_status.setStyleSheet("font-size: 15px; color: red; font-weight: bold;")
            self.log_message("No camera detected.")

        # Timer to update video frames
        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self.update_camera_feed)
        # *** Decreased from 50 ms to 30 ms (~33 fps)
        self.video_timer.start(30)

        # Let the main window accept arrow keys
        self.setFocusPolicy(Qt.StrongFocus)
        # Prevent the log console from stealing arrow keys (unless clicked)
        self.log_console.setFocusPolicy(Qt.ClickFocus)

        # Initialize Serial Reader Thread
        self.serial_thread = None

    def init_ui(self):
        """Builds the PyQt GUI layout."""
        self.setWindowTitle("Rover Controller")
        self.setGeometry(100, 100, 1080, 900)
        self.setFixedSize(1080, 900)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ---------------------
        # 3) TOP ROW LAYOUT
        # ---------------------
        top_row_layout = QHBoxLayout()
        main_layout.addLayout(top_row_layout)

        # 1) Camera Feed Frame
        camera_frame = QFrame()
        camera_frame.setFrameShape(QFrame.Box)
        camera_frame.setFrameShadow(QFrame.Raised)
        camera_frame.setStyleSheet("background-color: black;")
        camera_frame.setFixedSize(400, 300)
        top_row_layout.addWidget(camera_frame)

        # Create a label that fully covers the camera_frame
        self.camera_label = QLabel(camera_frame)
        self.camera_label.setGeometry(0, 0, 400, 300)
        self.camera_label.setStyleSheet("background-color: black;")
        # Enable scaled contents so the pixmap will fill the entire label
        self.camera_label.setScaledContents(True)

        # Optional: set an initial message
        self.camera_label.setText("Camera Feed")
        self.camera_label.setStyleSheet("color: white; font-size: 16px;")
        self.camera_label.setAlignment(Qt.AlignCenter)

        # 2) Rover Controller Frame
        rover_frame = QFrame()
        rover_frame.setFrameShape(QFrame.Box)
        rover_frame.setFrameShadow(QFrame.Sunken)
        rover_layout = QVBoxLayout(rover_frame)

        rover_label = QLabel("Rover Controller")
        rover_label.setStyleSheet("font-size: 16px;")
        rover_label.setAlignment(Qt.AlignCenter)
        rover_layout.addWidget(rover_label)

        self.rover_buttons = {}
        self.rover_button_styles = {}
        rover_buttons_layout = QGridLayout()
        rover_buttons_layout.setSpacing(4)

        rover_controls = {
            "Forward": ("↑", 0, 1),
            "TurnLeft": ("←", 1, 0),
            "Stop": ("S", 1, 1),
            "TurnRight": ("→", 1, 2),
            "Backward": ("↓", 2, 1),
        }

        # Define enhanced stylesheets with :pressed pseudo-state
        rover_button_style = """
            QPushButton {
                background-color: lightgray;
                border: 3px solid gray;
                border-radius: 10px;
                font-size: 18px;
            }
            QPushButton:pressed {
                background-color: lightgray;
                border: 4px solid blue;
            }
        """
        for label, (symbol, row, col) in rover_controls.items():
            btn = QPushButton(symbol)
            btn.setFixedSize(60, 60)
            btn.setStyleSheet(rover_button_style)  # Apply enhanced stylesheet
            self.rover_buttons[label] = btn
            rover_buttons_layout.addWidget(btn, row, col)

        rover_layout.addLayout(rover_buttons_layout)
        top_row_layout.addWidget(rover_frame)

        # 3) Camera Controller Frame
        camera_ctrl_frame = QFrame()
        camera_ctrl_frame.setFrameShape(QFrame.Box)
        camera_ctrl_frame.setFrameShadow(QFrame.Sunken)
        camera_layout = QVBoxLayout(camera_ctrl_frame)

        cam_ctrl_label = QLabel("Camera Controller")
        cam_ctrl_label.setStyleSheet("font-size: 16px;")
        cam_ctrl_label.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(cam_ctrl_label)

        self.camera_buttons = {}
        self.camera_button_styles = {}
        camera_buttons_layout = QGridLayout()
        camera_buttons_layout.setSpacing(4)

        camera_controls = {
            "Up": ("↑", 0, 1),
            "Left": ("←", 1, 0),
            "Center": ("C", 1, 1),
            "Right": ("→", 1, 2),
            "Down": ("↓", 2, 1),
        }

        # Define enhanced stylesheets with :pressed pseudo-state
        camera_button_style = """
            QPushButton {
                background-color: lightgray;
                border: 3px solid gray;
                border-radius: 10px;
                font-size: 18px;
            }
            QPushButton:pressed {
                background-color: lightgray;
                border: 4px solid blue;
            }
        """

        for label, (symbol, row, col) in camera_controls.items():
            btn = QPushButton(symbol)
            btn.setFixedSize(60, 60)
            btn.setStyleSheet(camera_button_style)  # Apply enhanced stylesheet
            self.camera_buttons[label] = btn
            camera_buttons_layout.addWidget(btn, row, col)

        camera_layout.addLayout(camera_buttons_layout)
        top_row_layout.addWidget(camera_ctrl_frame)

        # ---------------------
        # 4) MODE + LIGHT
        # ---------------------
        mode_buttons_layout = QHBoxLayout()
        main_layout.addLayout(mode_buttons_layout)

        self.manual_button = QPushButton("Manual")
        self.manual_button.setFixedSize(120, 40)
        self.manual_button.setStyleSheet(
            "background-color: red; color: white; border-radius: 5px; font-size: 14px;"
        )
        mode_buttons_layout.addWidget(self.manual_button)

        self.auto_button = QPushButton("Auto")
        self.auto_button.setFixedSize(120, 40)
        self.auto_button.setStyleSheet(
            "background-color: green; color: white; border-radius: 5px; font-size: 14px;"
        )
        mode_buttons_layout.addWidget(self.auto_button)

        self.light_on_button = QPushButton("Light ON")
        self.light_on_button.setFixedSize(100, 40)
        self.light_on_button.setStyleSheet(
            "background-color: yellow; color: black; border-radius: 5px; font-size: 14px;"
        )
        mode_buttons_layout.addWidget(self.light_on_button)

        self.light_off_button = QPushButton("Light OFF")
        self.light_off_button.setFixedSize(100, 40)
        self.light_off_button.setStyleSheet(
            "background-color: gray; color: white; border-radius: 5px; font-size: 14px;"
        )
        mode_buttons_layout.addWidget(self.light_off_button)

        # ---------------------
        # 5) Data Layout
        # ---------------------
        data_layout = QHBoxLayout()
        main_layout.addLayout(data_layout)

        self.gyro_label = QLabel("")
        self.gyro_label.setAlignment(Qt.AlignLeft)
        data_layout.addWidget(self.gyro_label)

        self.speed_label = QLabel("")
        self.speed_label.setAlignment(Qt.AlignLeft)
        data_layout.addWidget(self.speed_label)

        # ---------------------
        # 6) MAP
        # ---------------------
        map_frame = QFrame()
        map_frame.setFrameShape(QFrame.Box)
        map_frame.setFrameShadow(QFrame.Sunken)
        map_frame.setStyleSheet("background-color: white;")
        map_frame.setFixedSize(self.map_size[0], self.map_size[1])

        self.map_label = QLabel(map_frame)
        self.map_label.setPixmap(self.map_pixmap)
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setGeometry(0, 0, self.map_size[0], self.map_size[1])

        self.map_info_label = QLabel("2D Map Display", map_frame)
        self.map_info_label.setStyleSheet("font-size: 15px; color: black; background-color: transparent;")
        self.map_info_label.setGeometry(10, 10, 200, 20)

        main_layout.addWidget(map_frame, alignment=Qt.AlignCenter)

        # ---------------------
        # 7) CONNECTIVITY
        # ---------------------
        connectivity_layout = QVBoxLayout()
        main_layout.addLayout(connectivity_layout)

        # LoRa
        lora_layout = QHBoxLayout()
        self.lora_status = QLabel("LoRa: Not Connected")
        self.lora_status.setStyleSheet("font-size: 15px; color: red; font-weight: bold;")
        self.lora_status.setAlignment(Qt.AlignLeft)
        lora_layout.addWidget(self.lora_status)

        self.lora_com_port_combo = QComboBox()
        self.lora_com_port_combo.setFixedSize(150, 30)
        self.lora_com_port_combo.addItem("Select LoRa Port")
        self.populate_com_ports(self.lora_com_port_combo)
        self.lora_com_port_combo.currentIndexChanged.connect(self.update_lora_port)
        lora_layout.addWidget(self.lora_com_port_combo)
        connectivity_layout.addLayout(lora_layout)

        # Camera “connected” label
        self.camera_status = QLabel("Camera: Not Connected")
        self.camera_status.setStyleSheet("font-size: 15px; color: red; font-weight: bold;")
        self.camera_status.setAlignment(Qt.AlignLeft)
        connectivity_layout.addWidget(self.camera_status)

        self.mode_label = QLabel("Operating Mode : Manual")
        self.mode_label.setStyleSheet("font-size: 16px; color: black;")
        self.mode_label.setAlignment(Qt.AlignRight)
        connectivity_layout.addWidget(self.mode_label)

        # ---------------------
        # 8) LOG CONSOLE
        # ---------------------
        self.log_console = QTextEdit()
        self.log_console.setFixedHeight(80)
        self.log_console.setStyleSheet("background-color: #f5f5f5; font-size: 12px;")
        self.log_console.setReadOnly(True)
        main_layout.addWidget(self.log_console)

        self.setup_events()

        # Add a timer to refresh COM ports every 5 seconds
        #self.com_port_timer = QTimer()
        #self.com_port_timer.timeout.connect(lambda: self.populate_com_ports(self.lora_com_port_combo))
        # self.com_port_timer.start(5000)  # 5000 ms = 5 seconds

    def populate_com_ports(self, combo_box):
        """Populate LoRa COM port dropdown."""
        current_selection = combo_box.currentText()
        combo_box.blockSignals(True)
        combo_box.clear()
        combo_box.addItem("Select Port")
        ports = list_ports.comports()
        for port in ports:
            combo_box.addItem(port.device)
        # Restore previous selection if possible
        index = combo_box.findText(current_selection)
        if index >= 0:
            combo_box.setCurrentIndex(index)
        else:
            combo_box.setCurrentIndex(0)
        combo_box.blockSignals(False)

    def update_lora_port(self):
        """Handle selection of LoRa COM port."""
        selected_port = self.lora_com_port_combo.currentText()
        if selected_port == "Select Port":
            self.disconnect_lora()
            return

        # Disconnect existing thread if any
        self.disconnect_lora()

        # Start a new serial thread
        self.serial_thread = SerialReaderThread(port=selected_port)
        self.serial_thread.data_received.connect(self.process_received_data)
        self.serial_thread.error_occurred.connect(self.handle_serial_error)
        self.serial_thread.start()

        # Update status
        self.lora_status.setText(f"LoRa: Connected ({selected_port})")
        self.lora_status.setStyleSheet("font-size: 15px; color: green; font-weight: bold;")
        self.log_message(f"LoRa connected on port: {selected_port}")

    def disconnect_lora(self):
        """Disconnect the LoRa serial thread if it's running."""
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
            self.serial_thread = None
            self.lora_status.setText("LoRa: Not Connected")
            self.lora_status.setStyleSheet("font-size: 15px; color: red; font-weight: bold;")
            self.log_message("LoRa: Disconnected")

    def handle_serial_error(self, error_msg):
        """Handle serial communication errors emitted by the serial thread."""
        self.log_message(error_msg)
        self.disconnect_lora()

    def setup_events(self):
        # Manual/Auto
        self.manual_button.clicked.connect(self.activate_manual_mode)
        self.auto_button.clicked.connect(self.activate_auto_mode)

        # Light ON/OFF
        self.light_on_button.clicked.connect(lambda: self.send_single_command("lighton"))
        self.light_off_button.clicked.connect(lambda: self.send_single_command("lightoff"))

        # Rover Buttons
        self.rover_buttons["Forward"].clicked.connect(lambda: self.send_control_command("forward"))
        self.rover_buttons["Backward"].clicked.connect(lambda: self.send_control_command("backward"))
        self.rover_buttons["TurnLeft"].clicked.connect(lambda: self.send_control_command("left"))
        self.rover_buttons["TurnRight"].clicked.connect(lambda: self.send_control_command("right"))
        self.rover_buttons["Stop"].clicked.connect(lambda: self.send_control_command("stop"))

        # Camera Buttons
        self.camera_buttons["Up"].clicked.connect(lambda: self.send_control_command("camup"))
        self.camera_buttons["Down"].clicked.connect(lambda: self.send_control_command("camdown"))
        self.camera_buttons["Left"].clicked.connect(lambda: self.send_control_command("camleft"))
        self.camera_buttons["Right"].clicked.connect(lambda: self.send_control_command("camright"))
        self.camera_buttons["Center"].clicked.connect(lambda: self.send_control_command("camcenter"))

    def activate_manual_mode(self):
        self.manual_mode = True
        self.mode_label.setText("Operating Mode : Manual")
        self.manual_button.setStyleSheet(
            "background-color: red; color: white; border-radius: 5px; font-size: 16px;"
        )
        self.auto_button.setStyleSheet(
            "background-color: green; color: white; border-radius: 5px; font-size: 16px;"
        )
        self.send_single_command("manual")
        self.send_single_command("stop")
        self.set_controls_enabled(True)

    def activate_auto_mode(self):
        self.manual_mode = False
        self.mode_label.setText("Operating Mode : Auto")
        self.auto_button.setStyleSheet(
            "background-color: green; color: white; border-radius: 5px; font-size: 16px;"
        )
        self.manual_button.setStyleSheet(
            "background-color: red; color: white; border-radius: 5px; font-size: 16px;"
        )
        self.send_single_command("auto")
        self.set_controls_enabled(False)

    def set_controls_enabled(self, enabled):
        """Enable or disable rover and camera control buttons."""
        for btn in self.rover_buttons.values():
            btn.setEnabled(enabled)
        for btn in self.camera_buttons.values():
            btn.setEnabled(enabled)

    def send_single_command(self, cmd):
        if not self.manual_mode and cmd not in ["auto", "manual", "lighton", "lightoff"]:
            self.log_message(f"Ignoring '{cmd}' - in AUTO mode.")
            return
        self.log_message(f"Sending command: {cmd}")
        self.send_serial_command(cmd)

    def send_control_command(self, cmd):
        """Sends control commands only if in Manual Mode."""
        if self.manual_mode:
            self.log_message(f"Sending control command: {cmd}")
            self.send_serial_command(cmd)
        else:
            self.log_message(f"Ignored control command '{cmd}' - Currently in Auto Mode.")

    def send_serial_command(self, cmd):
        """Sends the command string over the serial connection if available."""
        if self.serial_thread and self.serial_thread.serial_connection and self.serial_thread.serial_connection.is_open:
            try:
                self.serial_thread.serial_connection.write((cmd + "\n").encode("utf-8"))
                self.log_message(f"Command '{cmd}' sent successfully.")
            except serial.SerialException as e:
                self.log_message(f"Error sending command: {e}")
                self.show_error_message("Send Error", f"Failed to send command: {e}")
            except Exception as e:
                # Catch any other error that might occur on write
                self.log_message(f"Unexpected error sending command: {e}\n{traceback.format_exc()}")
                
        else:
            self.log_message("LoRa port not open. Cannot send command.")

    def update_camera_feed(self):
        """Capture a frame, fill the camera_label fully (ignoring aspect ratio)."""
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        # Convert from BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Convert to QImage
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        self.camera_label.setPixmap(pixmap)

    def keyPressEvent(self, event):
        key = event.key()
        button = None  # To keep track of which button to animate

        # ROVER
        if key == Qt.Key_Up:
            self.send_single_command("forward")
            button = self.rover_buttons.get("Forward", None)
        elif key == Qt.Key_Down:
            self.send_single_command("backward")
            button = self.rover_buttons.get("Backward", None)
        elif key == Qt.Key_Left:
            self.send_single_command("left")
            button = self.rover_buttons.get("TurnLeft", None)
        elif key == Qt.Key_Right:
            self.send_single_command("right")
            button = self.rover_buttons.get("TurnRight", None)
        elif key == Qt.Key_0:
            self.send_single_command("stop")
            button = self.rover_buttons.get("Stop", None)

        # CAMERA
        elif key == Qt.Key_W:
            self.send_single_command("camup")
            button = self.camera_buttons.get("Up", None)
        elif key == Qt.Key_S:
            self.send_single_command("camdown")
            button = self.camera_buttons.get("Down", None)
        elif key == Qt.Key_A:
            self.send_single_command("camleft")
            button = self.camera_buttons.get("Left", None)
        elif key == Qt.Key_D:
            self.send_single_command("camright")
            button = self.camera_buttons.get("Right", None)
        elif key == Qt.Key_Z:
            self.send_single_command("camstop")
        elif key == Qt.Key_C:
            self.send_single_command("camcenter")
            button = self.camera_buttons.get("Center", None)
        elif key == Qt.Key_K:
            self.send_single_command("lighton")
        elif key == Qt.Key_L:
            self.send_single_command("lightoff")

        if button:
            original_style = button.styleSheet() if button else ""
            highlight_style = """
                QPushButton {
                    background-color: lightgrey;
                    border: 5px solid blue;
                    border-radius: 10px;
                    font-size: 18px;
                }
            """
            button.setStyleSheet(highlight_style)
            QTimer.singleShot(100, lambda: button.setStyleSheet(original_style))

        super().keyPressEvent(event)

    def log_message(self, msg):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_console.append(f"[{timestamp}] {msg}")

    def closeEvent(self, event):
        """Release camera and close serial port when closing the GUI."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.disconnect_lora()
        self.video_timer.stop()
        super().closeEvent(event)

    def is_significant_movement(self, new_lat, new_lon):
        """Return True if the new coordinates represent significant movement."""
        if not self.map_gps_data:
            return True
            
        last_lat, last_lon = self.map_gps_data[-1]
        
        # Calculate distance using simple Euclidean distance for estimation
        lat_diff = abs(new_lat - last_lat)
        lon_diff = abs(new_lon - last_lon)
        
        # Check if movement exceeds threshold
        return lat_diff > self.GPS_THRESHOLD or lon_diff > self.GPS_THRESHOLD
        
    def process_received_data(self, data):
        """
        Process the received LoRa data and update the map and log.
        Now includes GPS averaging/filtering to reduce jitter.
        """
        # Always strip whitespace/newlines
        data = data.strip()

        # Wrap parsing in a try/except to prevent crashes on malformed data
        try:
            if data.startswith("GPS:"):
                gps_data = data[4:].strip()  # e.g. "7.224368,79.975558"
                lat_str, lon_str = gps_data.split(",")
                lat, lon = float(lat_str), float(lon_str)

                # Add to GPS buffer for averaging
                self.gps_buffer.append((lat, lon))
                self.log_message(f"GPS Data: {lat}, {lon}")
                
                # Keep buffer at specified size
                if len(self.gps_buffer) > self.GPS_BUFFER_SIZE:
                    self.gps_buffer.pop(0)
                
                # Calculate average position
                avg_lat = sum(p[0] for p in self.gps_buffer) / len(self.gps_buffer)
                avg_lon = sum(p[1] for p in self.gps_buffer) / len(self.gps_buffer)
                
                # Update dynamic lat/lon boundaries for mapping
                self.min_lat = min(self.min_lat, avg_lat)
                self.max_lat = max(self.max_lat, avg_lat)
                self.min_lon = min(self.min_lon, avg_lon)
                self.max_lon = max(self.max_lon, avg_lon)
                
                # Store last position regardless
                self.last_gps_lat = avg_lat
                self.last_gps_lon = avg_lon
                
                # Only add a new map point if movement is significant
                if not self.map_gps_data or self.is_significant_movement(avg_lat, avg_lon):
                    self.map_gps_data.append((avg_lat, avg_lon))
                    self.log_message(f"GPS Data (filtered): {avg_lat}, {avg_lon}")

            elif data.startswith("OBSTACLE:"):
                obstacle_str = data.split(":", 1)[1].strip()  # e.g. "116"
                dist_mm = int(obstacle_str)
                current_lat = self.last_gps_lat
                current_lon = self.last_gps_lon

                self.map_obstacle_data.append((current_lat, current_lon, dist_mm))
                self.log_message(f"Obstacle => {dist_mm} mm from GPS({current_lat}, {current_lon})")

            else:
                # Unknown or unsupported data
                self.log_message(f"Unknown data received: {data}")

            # Redraw or refresh your 2D map
            self.update_map()

        except Exception as e:
            # If any unexpected parsing error occurs, log it instead of crashing
            self.log_message(
                f"Error parsing data '{data}': {e}\n{traceback.format_exc()}"
            )

    def update_map(self):
        """Redraw the 2D map based only on GPS data with last 3-4 digits of lat/lon."""
        self.map_pixmap.fill(Qt.white)
        painter = QPainter(self.map_pixmap)

        # Draw grid lines
        painter.setPen(QPen(Qt.lightGray, 1, Qt.DashLine))
        grid_spacing = 50
        for x in range(0, self.map_size[0], grid_spacing):
            painter.drawLine(x, 0, x, self.map_size[1])
        for y in range(0, self.map_size[1], grid_spacing):
            painter.drawLine(0, y, self.map_size[0], y)

        # Convert GPS coordinates to relative positions (using last 3-4 digits)
        def gps_to_pixel(lat, lon):
            # If we don't have any data yet, return center of the map
            if self.min_lat == float('inf') or self.max_lat == float('-inf'):
                return self.map_size[0] // 2, self.map_size[1] // 2
                
            # Calculate position using full GPS coordinates
            # Add small padding (0.0001) to avoid edge cases
            padding = 0.0001
            lat_range = max(self.max_lat - self.min_lat, padding)
            lon_range = max(self.max_lon - self.min_lon, padding)
            
            # Scale to fit the map
            x = int((lat - self.min_lat) / lat_range * (self.map_size[0] - 20)) + 10
            y = int((lon - self.min_lon) / lon_range * (self.map_size[1] - 20)) + 10
            
            # Ensure coordinates are within map bounds
            x = max(0, min(x, self.map_size[0]))
            y = max(0, min(y, self.map_size[1]))
            
            return x, y

        # Draw rover path in blue
        painter.setPen(QPen(Qt.blue, 2))
        if len(self.map_gps_data) > 0:
            for i in range(len(self.map_gps_data) - 1):
                lat1, lon1 = self.map_gps_data[i]
                lat2, lon2 = self.map_gps_data[i + 1]
                x1, y1 = gps_to_pixel(lat1, lon1)
                x2, y2 = gps_to_pixel(lat2, lon2)
                painter.drawLine(x1, y1, x2, y2)

        # Draw current GPS position in red
        if self.map_gps_data:
            lat_current, lon_current = self.map_gps_data[-1]
            x, y = gps_to_pixel(lat_current, lon_current)
            painter.setPen(Qt.NoPen)
            painter.setBrush(Qt.red)
            painter.drawEllipse(QPoint(x, y), 5, 5)  # Red dot for live position

        painter.end()
        self.map_label.setPixmap(self.map_pixmap)


    def show_error_message(self, title, message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Critical)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec_()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoverControlGUI()
    window.show()
    sys.exit(app.exec_())
