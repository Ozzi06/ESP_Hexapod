# !/usr/bin/env python3
import sys
import math
import time
import json
import socket
import threading
import requests  # For MJPEG stream

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QGroupBox, QFormLayout,
    QSizePolicy, QCheckBox, QTextEdit, QScrollArea, QStackedWidget, QComboBox, QSpinBox
)
from PySide6.QtCore import Qt, QTimer, Slot, QElapsedTimer, Signal, QObject, QThread
from PySide6.QtGui import QKeyEvent, QCloseEvent, QDoubleValidator, QTextCursor, QPixmap, QImage

try:
    from hexapod_comms_client import HexapodCommsClient, DEFAULT_ESP32_MJPEG_PORT

    LEG_COUNT = 6
except ImportError:
    print("CRITICAL ERROR: Could not find hexapod_comms_client.py. Ensure it is in the same directory or Python path.")
    sys.exit("Error: Could not find hexapod_comms_client.py.")

# --- Configuration ---
DEFAULT_ROBOT_TARGET_IP = "192.168.68.121"
DEFAULT_ROBOT_TCP_PORT = 5006
DEFAULT_ROBOT_UDP_PORT = 5005
# DEFAULT_ESP32_MJPEG_PORT is now imported from hexapod_comms_client
DEFAULT_GUI_TELEMETRY_LISTEN_IP = "0.0.0.0"
DEFAULT_GUI_TELEMETRY_LISTEN_PORT = 5007
DEFAULT_GUI_DISCOVERY_LISTEN_PORT = 5008
DEFAULT_UPDATE_FREQUENCY_HZ = 5.0
MIN_UPDATE_FREQUENCY_HZ = 5.0
MAX_UPDATE_FREQUENCY_HZ = 50.0

GUI_REF_MAX_LINEAR_SPEED = 8.0
GUI_REF_MAX_YAW_RATE = 0.3
GUI_REF_POSE_ADJUST_SPEED_LINEAR = 2.0
GUI_REF_POSE_ADJUST_SPEED_ANGULAR = math.radians(15.0)

GUI_REF_LINEAR_ACCEL_CMS2 = 10.0
GUI_REF_LINEAR_DECEL_CMS2 = 20.0
GUI_REF_YAW_ACCEL_DEGS2 = 30.0
GUI_REF_YAW_DECEL_DEGS2 = 60.0

GUI_REF_STEP_HEIGHT_CM = 3.0
GUI_REF_STEP_TIME_S = 1.0

LEG_NAMES = ["BR", "MR", "FR", "BL", "ML", "FL"]
DEFAULT_LEG_POS_FR_X_CM = 27.0
DEFAULT_LEG_POS_FR_Y_CM = 19.0
DEFAULT_LEG_POS_MR_X_CM = 32.0
DEFAULT_LEG_CORNER_EXT_CM = 0.0
DEFAULT_LEG_MIDDLE_EXT_CM = 0.0

ESP_LEG_MOUNTING_ANGLES = [
    -1 * math.pi / 4.0, 0 * math.pi / 4.0, +1 * math.pi / 4.0,
    -3 * math.pi / 4.0, +4 * math.pi / 4.0, +3 * math.pi / 4.0
]

# MJPEG Constants
MJPEG_BOUNDARY_BYTES = b"123456789000000000000987654321"  # From ESP32 streamer
FRAMESIZE_STR_QQVGA = "QQVGA"  # 160x120
FRAMESIZE_STR_HQVGA = "HQVGA"  # 240x176
FRAMESIZE_STR_QVGA = "QVGA"  # 320x240
FRAMESIZE_STR_CIF = "CIF"  # 400x296
FRAMESIZE_STR_VGA = "VGA"  # 640x480


class DiscoveryReceiverUDP(QObject):
    discovery_beacon_received_signal = Signal(dict)

    def __init__(self, listen_ip, listen_port):
        super().__init__()
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.running = False
        self.sock = None
        self.thread = None

    def start_listening(self):
        if self.running:
            return
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.listen_ip, self.listen_port))
            self.sock.settimeout(1.0)
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print(f"UDP Discovery listener started on {self.listen_ip}:{self.listen_port}")
        except Exception as e:
            print(f"[ERROR] UDP Discovery listener bind failed on {self.listen_ip}:{self.listen_port}: {e}")
            self.running = False
            if self.sock: self.sock.close(); self.sock = None

    def _listen_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                if data:
                    try:
                        json_data = json.loads(data.decode('utf-8'))
                        if json_data.get("type") == "hexapod_discovery_beacon":
                            self.discovery_beacon_received_signal.emit(json_data)
                    except json.JSONDecodeError:
                        print(f"[WARN UDP Discovery RX] JSON Decode Error. Data: {data[:100]}")
                    except Exception as e:
                        print(f"[WARN UDP Discovery RX] Processing error: {e}. Data: {data[:100]}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:  # Only log if we expected to be running
                    print(f"[ERROR UDP Discovery RX] Socket error: {e}")
                break  # Exit loop on other errors
        if self.sock:
            self.sock.close()
            self.sock = None
        print("UDP Discovery listener stopped.")

    def stop_listening(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.5)
            if self.thread.is_alive():
                print("[WARN] UDP Discovery listener thread did not join.")
        self.thread = None


class TelemetryReceiverUDP(QObject):
    telemetry_received_signal = Signal(dict)

    def __init__(self, listen_ip, listen_port):
        super().__init__()
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.running = False
        self.sock = None
        self.thread = None

    def start_listening(self):
        if self.running:
            return
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.listen_ip, self.listen_port))
            self.sock.settimeout(1.0)
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print(f"UDP Telemetry listener started on {self.listen_ip}:{self.listen_port}")
        except Exception as e:
            print(f"[ERROR] UDP Telemetry listener bind failed on {self.listen_ip}:{self.listen_port}: {e}")
            self.running = False
            if self.sock: self.sock.close(); self.sock = None

    def _listen_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
                if data:
                    try:
                        json_data = json.loads(data.decode('utf-8'))
                        self.telemetry_received_signal.emit(json_data)
                    except json.JSONDecodeError:
                        print(f"[WARN UDP RX] JSON Decode Error. Data: {data[:100]}")
                    except Exception as e:
                        print(f"[WARN UDP RX] Processing error: {e}. Data: {data[:100]}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:  # Only log if we expected to be running
                    print(f"[ERROR UDP RX] Socket error: {e}")
                break  # Exit loop on other errors
        if self.sock:
            self.sock.close()
            self.sock = None
        print("UDP Telemetry listener stopped.")

    def stop_listening(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.5)
            if self.thread.is_alive():
                print("[WARN] UDP Telemetry listener thread did not join.")
        self.thread = None


class MjpegStreamWorker(QObject):
    new_frame_signal = Signal(QImage)
    stream_error_signal = Signal(str)
    finished = Signal()
    _is_running = False
    _stream_url = ""
    _session = None  # Store session at class level for potential reuse/closure
    _response = None  # Store response at class level

    @Slot(str)
    def set_url(self, url):
        self._stream_url = url

    @Slot()
    def start_stream(self):
        if not self._stream_url:
            self.stream_error_signal.emit("Stream URL not set.")
            self.finished.emit()
            return

        self._is_running = True
        self.log_to_terminal(f"MJPEG Worker: Attempting to connect to stream: {self._stream_url}")
        self._session = requests.Session()
        try:
            headers = {'User-Agent': 'Python Hexapod GUI MJPEG Client'}
            self._response = self._session.get(self._stream_url, stream=True, timeout=(5, 10), headers=headers)
            self._response.raise_for_status()  # Raise HTTPError for bad responses (4xx or 5xx)
            self.log_to_terminal("MJPEG Worker: Connected to stream.")

            frame_data = b''
            full_boundary_bytes = b'--' + MJPEG_BOUNDARY_BYTES

            for chunk in self._response.iter_content(chunk_size=8192):  # iterate over data
                if not self._is_running:
                    self.log_to_terminal("MJPEG Worker: Stream stop requested in iter_content loop.")
                    break
                frame_data += chunk

                while self._is_running:  # Process all complete frames in the current buffer
                    boundary_start_pos = frame_data.find(full_boundary_bytes)
                    if boundary_start_pos == -1: break  # Need more data

                    boundary_line_end_pos = frame_data.find(b'\r\n', boundary_start_pos)
                    if boundary_line_end_pos == -1: boundary_line_end_pos = frame_data.find(b'\n', boundary_start_pos)
                    if boundary_line_end_pos == -1: break

                    headers_start_pos = boundary_line_end_pos + (
                        2 if frame_data[boundary_line_end_pos:boundary_line_end_pos + 2] == b'\r\n' else 1)

                    jpeg_data_start_pos_after_headers = frame_data.find(b'\r\n\r\n', headers_start_pos)
                    header_delimiter_len = 4
                    if jpeg_data_start_pos_after_headers == -1:
                        jpeg_data_start_pos_after_headers = frame_data.find(b'\n\n', headers_start_pos)
                        header_delimiter_len = 2
                    if jpeg_data_start_pos_after_headers == -1: break

                    actual_jpeg_data_start = jpeg_data_start_pos_after_headers + header_delimiter_len
                    next_boundary_start_pos = frame_data.find(full_boundary_bytes, actual_jpeg_data_start)
                    if next_boundary_start_pos == -1: break

                    jpeg_bytes = frame_data[actual_jpeg_data_start:next_boundary_start_pos]
                    frame_data = frame_data[next_boundary_start_pos:]

                    if not self._is_running:  # Check again after parsing, before emitting
                        self.log_to_terminal("MJPEG Worker: Stream stop requested before emitting frame.")
                        break

                    if jpeg_bytes:
                        try:
                            q_image = QImage()
                            if q_image.loadFromData(jpeg_bytes, "JPEG") and not q_image.isNull():
                                self.new_frame_signal.emit(q_image)
                            # else: self.log_to_terminal(f"MJPEG Worker: QImage.loadFromData failed or image isNull. Bytes: {len(jpeg_bytes)}")
                        except Exception as e:
                            self.log_to_terminal(f"MJPEG Worker: Error converting JPEG to QImage: {e}")

                if not self._is_running:  # If inner loop broke due to stop_stream
                    self.log_to_terminal("MJPEG Worker: Inner loop broken due to stop request, breaking outer.")
                    break

            # After for loop finishes or breaks:
            if self._is_running and self._response:  # This means loop exited for reason other than stop_stream or normal end
                self.stream_error_signal.emit("Stream ended or connection lost by server.")
            self.log_to_terminal("MJPEG Worker: Stream loop finished.")

        except requests.exceptions.ConnectionError as e:
            if self._is_running: self.stream_error_signal.emit(
                f"Stream Connection Error: {e}. Check IP/Port: {self._stream_url}")
        except requests.exceptions.Timeout:
            if self._is_running: self.stream_error_signal.emit(f"Stream Connection Timeout for {self._stream_url}")
        except requests.exceptions.RequestException as e:  # Catches HTTPError from raise_for_status too
            if self._is_running: self.stream_error_signal.emit(f"Stream request error: {e} for {self._stream_url}")
        except Exception as e:  # Catch any other unexpected error in worker
            if self._is_running: self.stream_error_signal.emit(f"MJPEG Worker: Unexpected error: {e}")
            self.log_to_terminal(f"MJPEG Worker: CRITICAL UNEXPECTED ERROR: {e}")
        finally:
            self.log_to_terminal("MJPEG Worker: In finally block.")
            if self._response:
                self._response.close()
                self._response = None
                self.log_to_terminal("MJPEG Worker: Response closed.")
            if self._session:
                self._session.close()
                self._session = None
                self.log_to_terminal("MJPEG Worker: Session closed.")
            self._is_running = False  # Ensure flag is reset
            self.finished.emit()
            self.log_to_terminal("MJPEG Worker: Cleanup complete, finished emitted.")

    def stop_stream(self):
        self.log_to_terminal("MJPEG Worker: stop_stream() called.")
        self._is_running = False
        # Attempt to close response/session here might be risky if called from different thread
        # Rely on the main loop checking _is_running and cleaning up in finally.

    def log_to_terminal(self, message):  # Helper for worker logging
        print(message)  # Worker logs to console


class HexapodControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hexapod Controller GUI v2.2.1 (TCP/UDP + Discovery + MJPEG)")
        self.setGeometry(30, 30, 1600, 950)  # Adjusted width for more columns

        self.comms_client = None
        self.udp_intent_timer = QTimer(self)
        self.udp_intent_timer.timeout.connect(self.send_active_intents_udp)

        self.udp_telemetry_receiver = TelemetryReceiverUDP(
            DEFAULT_GUI_TELEMETRY_LISTEN_IP, DEFAULT_GUI_TELEMETRY_LISTEN_PORT
        )
        self.udp_telemetry_receiver.telemetry_received_signal.connect(self.handle_udp_telemetry)

        self.udp_discovery_receiver = DiscoveryReceiverUDP(
            DEFAULT_GUI_TELEMETRY_LISTEN_IP, DEFAULT_GUI_DISCOVERY_LISTEN_PORT
        )
        self.udp_discovery_receiver.discovery_beacon_received_signal.connect(self.handle_discovery_beacon)

        self._stream_thread = None
        self._stream_worker = None

        # Telemetry Labels
        self.actual_battery_voltage_label = QLabel("N/A")
        self.actual_robot_status_label = QLabel("N/A")
        self.actual_velocity_x_label = QLabel("N/A")
        self.actual_velocity_y_label = QLabel("N/A")
        self.actual_yaw_rate_label = QLabel("N/A")
        self.actual_pos_x_label = QLabel("N/A")
        self.actual_pos_y_label = QLabel("N/A")
        self.actual_pos_z_label = QLabel("N/A")
        self.actual_orient_w_label = QLabel("N/A")
        self.actual_orient_x_label = QLabel("N/A")
        self.actual_orient_y_label = QLabel("N/A")
        self.actual_orient_z_label = QLabel("N/A")
        self.actual_gait_h_label = QLabel("N/A")
        self.actual_gait_t_label = QLabel("N/A")
        self.actual_walk_status_label = QLabel("N/A")
        self.rtt_label = QLabel("RTT: N/A ms")

        self.debug_foot_pos_group = None
        self.debug_fp_walk_labels = [{'x': QLabel("X: N/A"), 'y': QLabel("Y: N/A"), 'z': QLabel("Z: N/A")} for _ in
                                     range(LEG_COUNT)]

        self.all_config_spinboxes = []

        self.keys_pressed = set()
        self.loco_intent_vx_factor = 0.0;
        self.loco_intent_vy_factor = 0.0;
        self.loco_intent_yaw_factor = 0.0
        self.pose_adjust_intent_offset_x = 0.0;
        self.pose_adjust_intent_offset_y = 0.0;
        self.pose_adjust_intent_offset_z = 0.0
        self.pose_adjust_intent_pitch = 0.0;
        self.pose_adjust_intent_roll = 0.0;
        self.pose_adjust_intent_body_yaw = 0.0
        self.centering_xy_active = False;
        self.centering_orientation_active = False

        self._init_ui()
        self.set_ui_for_disconnected_state()
        self.update_gui_calculated_base_pos_display()

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Connection Group ---
        connection_group = QGroupBox("Connection")
        connection_layout = QHBoxLayout(connection_group)
        self.discover_button = QPushButton("Discover Robot")
        self.discover_button.clicked.connect(self.start_robot_discovery)
        connection_layout.addWidget(self.discover_button)
        connection_layout.addWidget(QLabel("Robot IP:"))
        self.robot_ip_input = QLineEdit(DEFAULT_ROBOT_TARGET_IP)
        connection_layout.addWidget(self.robot_ip_input)
        connection_layout.addWidget(QLabel("TCP Port:"))
        self.robot_tcp_port_input = QLineEdit(str(DEFAULT_ROBOT_TCP_PORT));
        self.robot_tcp_port_input.setFixedWidth(60)
        connection_layout.addWidget(self.robot_tcp_port_input)
        connection_layout.addWidget(QLabel("UDP Port:"))
        self.robot_udp_port_input = QLineEdit(str(DEFAULT_ROBOT_UDP_PORT));
        self.robot_udp_port_input.setFixedWidth(60)
        connection_layout.addWidget(self.robot_udp_port_input)
        connection_layout.addWidget(QLabel("MJPEG Port:"))
        self.robot_mjpeg_port_input = QLineEdit(str(DEFAULT_ESP32_MJPEG_PORT));
        self.robot_mjpeg_port_input.setFixedWidth(60)
        connection_layout.addWidget(self.robot_mjpeg_port_input)
        connection_layout.addWidget(QLabel("Intent Freq (Hz):"))
        self.update_freq_input = QDoubleSpinBox();
        self.update_freq_input.setRange(MIN_UPDATE_FREQUENCY_HZ, MAX_UPDATE_FREQUENCY_HZ)
        self.update_freq_input.setValue(DEFAULT_UPDATE_FREQUENCY_HZ);
        self.update_freq_input.setDecimals(1)
        self.update_freq_input.valueChanged.connect(self.update_udp_intent_timer_interval)
        connection_layout.addWidget(self.update_freq_input)
        self.connect_button = QPushButton("Connect");
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_button)
        self.fetch_state_button = QPushButton("Fetch Robot State");
        self.fetch_state_button.clicked.connect(self.request_full_state_from_robot)
        connection_layout.addWidget(self.fetch_state_button)
        self.connection_status_label = QLabel("Status: Disconnected")
        connection_layout.addWidget(self.connection_status_label)
        connection_layout.addWidget(self.rtt_label);
        self.rtt_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        connection_layout.addStretch(1)
        main_layout.addWidget(connection_group)

        # --- Main Content Area (GridLayout) ---
        main_content_widget = QWidget()
        content_layout = QGridLayout(main_content_widget)  # Use QGridLayout

        # --- Video Box Group (Spans Col 0, 1; Row 0) ---
        video_box_group = QGroupBox("Camera Stream")
        video_box_layout = QVBoxLayout(video_box_group)
        self.video_stacked_widget = QStackedWidget()

        # Video Config Widget (Index 0 in StackedWidget)
        self.video_config_widget = QWidget()
        video_config_form = QFormLayout(self.video_config_widget)
        self.video_resolution_combo = QComboBox()
        self.video_resolution_combo.addItems(
            [FRAMESIZE_STR_QQVGA, FRAMESIZE_STR_HQVGA, FRAMESIZE_STR_QVGA, FRAMESIZE_STR_CIF, FRAMESIZE_STR_VGA])
        self.video_resolution_combo.setCurrentText(FRAMESIZE_STR_QQVGA)
        video_config_form.addRow("Resolution:", self.video_resolution_combo)
        self.video_quality_spin = QSpinBox();
        self.video_quality_spin.setRange(0, 63);
        self.video_quality_spin.setValue(20)
        video_config_form.addRow("JPEG Quality (0-63), 63 is worst:", self.video_quality_spin)
        self.video_fps_spin = QSpinBox();
        self.video_fps_spin.setRange(0, 30);
        self.video_fps_spin.setValue(10)
        video_config_form.addRow("FPS Limit:", self.video_fps_spin)
        self.video_apply_config_button = QPushButton("Apply Camera Config (TCP)");
        self.video_apply_config_button.clicked.connect(self.send_camera_config_command_tcp)
        video_config_form.addRow(self.video_apply_config_button)
        self.video_start_stream_button = QPushButton("Start Stream (TCP)");
        self.video_start_stream_button.clicked.connect(self.send_camera_start_stream_command_tcp)
        video_config_form.addRow(self.video_start_stream_button)
        self.video_stacked_widget.addWidget(self.video_config_widget)

        # Video Stream Widget (Index 1 in StackedWidget)
        self.video_stream_widget = QWidget()
        video_stream_layout = QVBoxLayout(self.video_stream_widget)
        self.video_label = QLabel("Video stream will appear here.")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(320, 240)
        self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video_label.setStyleSheet("background-color: black; color: white; border: 1px solid gray;")
        video_stream_layout.addWidget(self.video_label, 1)  # Give more stretch to label
        self.video_stop_stream_button = QPushButton("Stop Stream (TCP)");
        self.video_stop_stream_button.clicked.connect(self.send_camera_stop_stream_command_tcp)
        video_stream_layout.addWidget(self.video_stop_stream_button)
        self.video_stacked_widget.addWidget(self.video_stream_widget)

        video_box_layout.addWidget(self.video_stacked_widget)
        self.video_stacked_widget.setCurrentIndex(0)  # Start with config view
        content_layout.addWidget(video_box_group, 0, 0, 1, 2)  # Row 0, Col 0, RowSpan 1, ColSpan 2

        # --- Column 1 (Shortened) ---
        col1_widget = QWidget();
        col1_layout = QVBoxLayout(col1_widget)
        sys_cmd_group = QGroupBox("System Commands");
        sys_cmd_form = QFormLayout(sys_cmd_group)
        self.pwm_reset_button = QPushButton("Re-initialize PWM Drivers");
        self.pwm_reset_button.clicked.connect(self.send_pwm_reset_command_tcp)
        sys_cmd_form.addRow(self.pwm_reset_button)
        col1_layout.addWidget(sys_cmd_group)
        instructions_group = QGroupBox("Keyboard Controls (Intents via UDP)")
        instr_text = ("W/S: Fwd/Back\nA/D: Strafe L/R\nQ/E: Turn L/R\nSPACE: Toggle Walk Cmd (TCP)\n"
                      "--- Body Pose Adjust ---\nArrows: Move Body XY\nShift+Arrows: Move Body Z\n"
                      "I/K: Pitch\nJ/L: Roll\nU/O: Body Yaw\n"
                      "C: Center XY (Hold)\nX: Center Orientation (Hold)")
        instr_label = QLabel(instr_text);
        instr_label.setWordWrap(True)
        instr_layout_v = QVBoxLayout(instructions_group);
        instr_layout_v.addWidget(instr_label)
        col1_layout.addWidget(instructions_group)
        col1_layout.addStretch(1)
        content_layout.addWidget(col1_widget, 1, 0)  # Row 1, Col 0

        # --- Column 2 (Shortened) - Telemetry Subscriptions ---
        col2_widget = QWidget();
        col2_layout = QVBoxLayout(col2_widget)
        telemetry_config_group = QGroupBox("Telemetry Subscriptions")
        telemetry_config_form = QFormLayout(telemetry_config_group)
        self.sub_battery_check = QCheckBox("Battery (TCP)");
        self.sub_battery_check.setChecked(True)
        self.sub_battery_interval = QLineEdit("1000");
        self.sub_battery_interval.setFixedWidth(50)
        telemetry_config_form.addRow(self.sub_battery_check, self.create_interval_layout(self.sub_battery_interval))
        self.sub_robot_status_check = QCheckBox("Robot Status (TCP)");
        self.sub_robot_status_check.setChecked(True)
        self.sub_robot_status_interval = QLineEdit("5000");
        self.sub_robot_status_interval.setFixedWidth(50)
        telemetry_config_form.addRow(self.sub_robot_status_check,
                                     self.create_interval_layout(self.sub_robot_status_interval))
        self.sub_robot_state_actual_check = QCheckBox("Robot State (UDP)");
        self.sub_robot_state_actual_check.setChecked(True)
        self.sub_robot_state_actual_interval = QLineEdit("200");
        self.sub_robot_state_actual_interval.setFixedWidth(50)
        telemetry_config_form.addRow(self.sub_robot_state_actual_check,
                                     self.create_interval_layout(self.sub_robot_state_actual_interval))
        self.sub_debug_foot_pos_check = QCheckBox("Debug Foot Pos (UDP)");
        self.sub_debug_foot_pos_check.setChecked(False)
        self.sub_debug_foot_pos_interval = QLineEdit("200");
        self.sub_debug_foot_pos_interval.setFixedWidth(50)
        telemetry_config_form.addRow(self.sub_debug_foot_pos_check,
                                     self.create_interval_layout(self.sub_debug_foot_pos_interval))
        update_subs_button = QPushButton("Update Telemetry Subscriptions (TCP)");
        update_subs_button.clicked.connect(self.send_client_settings_to_robot)
        telemetry_config_form.addRow(update_subs_button)
        col2_layout.addWidget(telemetry_config_group)
        col2_layout.addStretch(1)
        content_layout.addWidget(col2_widget, 1, 1)  # Row 1, Col 1

        # --- Column 3 (Full Height) - Leg Geometry ---
        col3_widget = QWidget();
        col3_layout = QVBoxLayout(col3_widget)
        leg_base_geom_group = QGroupBox("Config: Leg Base Geometry (Abstract, cm)")
        leg_base_geom_form = QFormLayout(leg_base_geom_group)
        self.cfg_leg_front_corner_x = QDoubleSpinBox();
        self.cfg_leg_front_corner_x.setRange(-50, 50);
        self.cfg_leg_front_corner_x.setDecimals(1)
        leg_base_geom_form.addRow("Front Corner X:", self.cfg_leg_front_corner_x)
        self.cfg_leg_front_corner_y = QDoubleSpinBox();
        self.cfg_leg_front_corner_y.setRange(-50, 50);
        self.cfg_leg_front_corner_y.setDecimals(1)
        leg_base_geom_form.addRow("Front Corner Y:", self.cfg_leg_front_corner_y)
        self.cfg_leg_middle_side_x = QDoubleSpinBox();
        self.cfg_leg_middle_side_x.setRange(-50, 50);
        self.cfg_leg_middle_side_x.setDecimals(1)
        leg_base_geom_form.addRow("Middle Side X:", self.cfg_leg_middle_side_x)
        self.cfg_leg_corner_ext = QDoubleSpinBox();
        self.cfg_leg_corner_ext.setRange(-10, 10);
        self.cfg_leg_corner_ext.setDecimals(1)
        leg_base_geom_form.addRow("Corner Legs Ext:", self.cfg_leg_corner_ext)
        self.cfg_leg_middle_ext = QDoubleSpinBox();
        self.cfg_leg_middle_ext.setRange(-10, 10);
        self.cfg_leg_middle_ext.setDecimals(1)
        leg_base_geom_form.addRow("Middle Legs Ext:", self.cfg_leg_middle_ext)
        col3_layout.addWidget(leg_base_geom_group)
        self.all_config_spinboxes.extend(
            [self.cfg_leg_front_corner_x, self.cfg_leg_front_corner_y, self.cfg_leg_middle_side_x,
             self.cfg_leg_corner_ext, self.cfg_leg_middle_ext])
        for sb in [self.cfg_leg_front_corner_x, self.cfg_leg_front_corner_y, self.cfg_leg_middle_side_x,
                   self.cfg_leg_corner_ext, self.cfg_leg_middle_ext]:
            sb.valueChanged.connect(self.send_leg_geometry_configs_auto)
        gui_base_pos_group = QGroupBox("GUI Calculated Base Foot Positions (Walk Frame cm)")
        gui_base_pos_form = QFormLayout(gui_base_pos_group)
        self.gui_base_pos_labels = [{'x': QLabel("X:0.0"), 'y': QLabel("Y:0.0"), 'z': QLabel("Z:0.0")} for _ in
                                    range(LEG_COUNT)]
        for i in range(LEG_COUNT):
            rowLayout = QHBoxLayout();
            rowLayout.addWidget(self.gui_base_pos_labels[i]['x']);
            rowLayout.addWidget(self.gui_base_pos_labels[i]['y']);
            rowLayout.addWidget(self.gui_base_pos_labels[i]['z'])
            gui_base_pos_form.addRow(f"Leg {i} ({LEG_NAMES[i]}):", rowLayout)
        col3_layout.addWidget(gui_base_pos_group)
        col3_layout.addStretch(1)
        content_layout.addWidget(col3_widget, 0, 2, 2, 1)  # Row 0, Col 2, RowSpan 2, ColSpan 1

        # --- Column 4 (Full Height) - Speeds, Accel, Gait Configs ---
        col4_widget = QWidget();
        col4_layout = QVBoxLayout(col4_widget)
        config_speeds_group = QGroupBox("Config: Speeds & Rates")
        config_speeds_form = QFormLayout(config_speeds_group)
        self.cfg_max_linear_speed_input = QDoubleSpinBox();
        self.cfg_max_linear_speed_input.setRange(1, 50)
        config_speeds_form.addRow("Max Linear Speed (cm/s):", self.cfg_max_linear_speed_input)
        self.cfg_max_yaw_rate_input = QDoubleSpinBox();
        self.cfg_max_yaw_rate_input.setRange(0.01, math.pi);
        self.cfg_max_yaw_rate_input.setDecimals(2)
        config_speeds_form.addRow("Max Yaw Rate (rad/s):", self.cfg_max_yaw_rate_input)
        self.cfg_pose_linear_speed_input = QDoubleSpinBox();
        self.cfg_pose_linear_speed_input.setRange(0.1, 1000)
        config_speeds_form.addRow("Pose Adjust Lin Spd (cm/s):", self.cfg_pose_linear_speed_input)
        self.cfg_pose_angular_speed_input = QDoubleSpinBox();
        self.cfg_pose_angular_speed_input.setRange(1, 90)
        config_speeds_form.addRow("Pose Adjust Ang Spd (deg/s):", self.cfg_pose_angular_speed_input)
        col4_layout.addWidget(config_speeds_group)
        self.all_config_spinboxes.extend(
            [self.cfg_max_linear_speed_input, self.cfg_max_yaw_rate_input, self.cfg_pose_linear_speed_input,
             self.cfg_pose_angular_speed_input])
        for sb in [self.cfg_max_linear_speed_input, self.cfg_max_yaw_rate_input, self.cfg_pose_linear_speed_input,
                   self.cfg_pose_angular_speed_input]:
            sb.valueChanged.connect(self.send_movement_configs_auto)
        config_accel_group = QGroupBox("Config: Acceleration")
        config_accel_form = QFormLayout(config_accel_group)
        self.cfg_linear_accel_input = QDoubleSpinBox();
        self.cfg_linear_accel_input.setRange(1.0, 500.0);
        self.cfg_linear_accel_input.setDecimals(1)
        config_accel_form.addRow("Linear Accel (cm/s²):", self.cfg_linear_accel_input)
        self.cfg_linear_decel_input = QDoubleSpinBox();
        self.cfg_linear_decel_input.setRange(1.0, 1000.0);
        self.cfg_linear_decel_input.setDecimals(1)
        config_accel_form.addRow("Linear Decel (cm/s²):", self.cfg_linear_decel_input)
        self.cfg_yaw_accel_input = QDoubleSpinBox();
        self.cfg_yaw_accel_input.setRange(0.1, 360.0);
        self.cfg_yaw_accel_input.setDecimals(1)
        config_accel_form.addRow("Yaw Accel (deg/s²):", self.cfg_yaw_accel_input)
        self.cfg_yaw_decel_input = QDoubleSpinBox();
        self.cfg_yaw_decel_input.setRange(0.1, 720.0);
        self.cfg_yaw_decel_input.setDecimals(1)
        config_accel_form.addRow("Yaw Decel (deg/s²):", self.cfg_yaw_decel_input)
        col4_layout.addWidget(config_accel_group)
        self.all_config_spinboxes.extend(
            [self.cfg_linear_accel_input, self.cfg_linear_decel_input, self.cfg_yaw_accel_input,
             self.cfg_yaw_decel_input])
        for sb in [self.cfg_linear_accel_input, self.cfg_linear_decel_input, self.cfg_yaw_accel_input,
                   self.cfg_yaw_decel_input]:
            sb.valueChanged.connect(self.send_accel_configs_auto)
        config_gait_group = QGroupBox("Config: Gait Parameters")
        config_gait_form = QFormLayout(config_gait_group)
        self.cfg_step_height_input = QDoubleSpinBox();
        self.cfg_step_height_input.setRange(0, 10)
        config_gait_form.addRow("Step Height (cm):", self.cfg_step_height_input)
        self.cfg_step_time_input = QDoubleSpinBox();
        self.cfg_step_time_input.setRange(0.1, 5)
        config_gait_form.addRow("Step Time (s):", self.cfg_step_time_input)
        col4_layout.addWidget(config_gait_group)
        self.all_config_spinboxes.extend([self.cfg_step_height_input, self.cfg_step_time_input])
        for sb in [self.cfg_step_height_input, self.cfg_step_time_input]:
            sb.valueChanged.connect(self.send_gait_configs_auto)
        col4_layout.addStretch(1)
        content_layout.addWidget(col4_widget, 0, 3, 2, 1)  # Row 0, Col 3, RowSpan 2, ColSpan 1

        # --- Column 5 (Full Height) - Robot Telemetry & State ---
        col5_widget = QWidget();
        col5_layout = QVBoxLayout(col5_widget)
        robot_telemetry_group = QGroupBox("Robot Telemetry (from TCP)")  # Renamed and moved
        robot_telemetry_form = QFormLayout(robot_telemetry_group)
        robot_telemetry_form.addRow("Battery Voltage (do not trust):", self.actual_battery_voltage_label)
        robot_telemetry_form.addRow("Robot Status:", self.actual_robot_status_label)
        col5_layout.addWidget(robot_telemetry_group)
        actual_state_group = QGroupBox("Robot Actual State (from Telemetry)")
        actual_state_form = QFormLayout(actual_state_group)
        actual_state_form.addRow("Cmd Walk Active:", self.actual_walk_status_label)
        actual_state_form.addRow("Actual Vel X (UDP):", self.actual_velocity_x_label)
        actual_state_form.addRow("Actual Vel Y (UDP):", self.actual_velocity_y_label)
        actual_state_form.addRow("Actual Yaw Rate (UDP):", self.actual_yaw_rate_label)
        actual_state_form.addRow("Actual Pos X (UDP):", self.actual_pos_x_label)
        actual_state_form.addRow("Actual Pos Y (UDP):", self.actual_pos_y_label)
        actual_state_form.addRow("Actual Pos Z (UDP):", self.actual_pos_z_label)
        actual_state_form.addRow("Actual Orient W (UDP):", self.actual_orient_w_label)
        actual_state_form.addRow("Actual Orient X (UDP):", self.actual_orient_x_label)
        actual_state_form.addRow("Actual Orient Y (UDP):", self.actual_orient_y_label)
        actual_state_form.addRow("Actual Orient Z (UDP):", self.actual_orient_z_label)
        actual_state_form.addRow("Actual Gait Height (UDP):", self.actual_gait_h_label)
        actual_state_form.addRow("Actual Gait Time (UDP):", self.actual_gait_t_label)
        col5_layout.addWidget(actual_state_group)
        self.debug_foot_pos_group = QGroupBox("Debug: Foot Positions (UDP - Walk Frame cm)")
        debug_fp_form = QFormLayout(self.debug_foot_pos_group)
        for i in range(LEG_COUNT):
            row_layout = QHBoxLayout();
            row_layout.addWidget(self.debug_fp_walk_labels[i]['x']);
            row_layout.addWidget(self.debug_fp_walk_labels[i]['y']);
            row_layout.addWidget(self.debug_fp_walk_labels[i]['z'])
            debug_fp_form.addRow(f"Leg {i} ({LEG_NAMES[i]}):", row_layout)
        self.debug_foot_pos_group.setVisible(False)
        self.sub_debug_foot_pos_check.stateChanged.connect(
            lambda state: self.debug_foot_pos_group.setVisible(state == Qt.CheckState.Checked.value))
        col5_layout.addWidget(self.debug_foot_pos_group)
        col5_layout.addStretch(1)
        content_layout.addWidget(col5_widget, 0, 4, 2, 1)  # Row 0, Col 4, RowSpan 2, ColSpan 1

        # Set column stretch factors for the QGridLayout
        content_layout.setColumnStretch(0, 2)  # Col 1 (short)
        content_layout.setColumnStretch(1, 2)  # Col 2 (short)
        content_layout.setColumnStretch(2, 3)  # Col 3 (full)
        content_layout.setColumnStretch(3, 3)  # Col 4 (full)
        content_layout.setColumnStretch(4, 3)  # Col 5 (full)

        main_layout.addWidget(main_content_widget)

        # --- Terminal Log Area ---
        self.terminal_log_area = QTextEdit();
        self.terminal_log_area.setReadOnly(True)
        self.terminal_log_area.setFixedHeight(100)
        self.terminal_log_area.setStyleSheet(
            "background-color: #f0f0f0; color: #333; font-family: Consolas, monospace;")
        main_layout.addWidget(self.terminal_log_area)

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def create_interval_layout(self, line_edit_widget):
        layout = QHBoxLayout();
        layout.addWidget(QLabel("Interval(ms):"));
        layout.addWidget(line_edit_widget)
        return layout

    def log_to_terminal(self, message: str):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.terminal_log_area.append(f"[{timestamp}] {message}")
        self.terminal_log_area.moveCursor(QTextCursor.MoveOperation.End)
        print(f"GUI_LOG: {message}")  # Also print to console for easier debugging

    def set_ui_for_disconnected_state(self):
        self.log_to_terminal("UI set to disconnected state.")
        self.robot_ip_input.setEnabled(True);
        self.robot_tcp_port_input.setEnabled(True)
        self.robot_udp_port_input.setEnabled(True);
        self.robot_mjpeg_port_input.setEnabled(True)
        self.discover_button.setEnabled(True)
        self.connect_button.setText("Connect");
        self.connect_button.setEnabled(True)
        self.fetch_state_button.setEnabled(False)
        self.connection_status_label.setText("Status: Disconnected");
        self.connection_status_label.setStyleSheet("color: black")
        self.rtt_label.setText("RTT: N/A ms")

        for sb in self.all_config_spinboxes:
            sb.blockSignals(True);
            sb.setEnabled(False);
            sb.setSpecialValueText("N/A")
            sb.setValue(sb.minimum() if not (sb.minimum() <= 0.0 and sb.maximum() >= 0.0) else 0.0)
            sb.blockSignals(False)

        self.pwm_reset_button.setEnabled(False)
        # Camera UI
        self.video_stacked_widget.setCurrentIndex(0)  # Show config
        self.video_apply_config_button.setEnabled(False);
        self.video_start_stream_button.setEnabled(False)
        self.video_resolution_combo.setEnabled(False);
        self.video_quality_spin.setEnabled(False);
        self.video_fps_spin.setEnabled(False)
        self.video_label.setText("Stream offline.");
        self.video_label.setStyleSheet("background-color: black; color: gray;")

        for label_dict_list in [self.debug_fp_walk_labels]:  # Add other lists if any
            for item_dict in label_dict_list:
                for k in item_dict: item_dict[k].setText(f"{k.upper()}: N/A")
        for label in [self.actual_battery_voltage_label, self.actual_robot_status_label, self.actual_velocity_x_label,
                      self.actual_velocity_y_label, self.actual_yaw_rate_label, self.actual_pos_x_label,
                      self.actual_pos_y_label, self.actual_pos_z_label, self.actual_orient_w_label,
                      self.actual_orient_x_label, self.actual_orient_y_label, self.actual_orient_z_label,
                      self.actual_gait_h_label, self.actual_gait_t_label, self.actual_walk_status_label]:
            label.setText("N/A")
        self.actual_walk_status_label.setStyleSheet("color: gray")

    def set_ui_for_connected_but_unsynced_state(self):
        self.log_to_terminal("UI set to connected but unsynced state.")
        self.robot_ip_input.setEnabled(False);
        self.robot_tcp_port_input.setEnabled(False)
        self.robot_udp_port_input.setEnabled(False);
        self.robot_mjpeg_port_input.setEnabled(False)
        self.discover_button.setEnabled(False)
        self.connect_button.setText("Disconnect");
        self.connect_button.setEnabled(True)
        self.fetch_state_button.setEnabled(True)
        self.rtt_label.setText("RTT: Pinging...")

        for sb in self.all_config_spinboxes:
            sb.blockSignals(True);
            sb.setEnabled(False);
            sb.setSpecialValueText("N/A")
            sb.setValue(sb.minimum() if not (sb.minimum() <= 0.0 and sb.maximum() >= 0.0) else 0.0)
            sb.blockSignals(False)
        self.pwm_reset_button.setEnabled(True)
        # Camera UI
        self.video_stacked_widget.setCurrentIndex(0)  # Show config
        self.video_apply_config_button.setEnabled(True);
        self.video_start_stream_button.setEnabled(True)
        self.video_resolution_combo.setEnabled(True);
        self.video_quality_spin.setEnabled(True);
        self.video_fps_spin.setEnabled(True)

    def set_ui_for_synced_state(self):
        self.log_to_terminal("UI set to synced state.")
        self.fetch_state_button.setEnabled(True)
        for sb in self.all_config_spinboxes:
            sb.setEnabled(True);
            sb.setSpecialValueText("")
        self.pwm_reset_button.setEnabled(True)
        # Camera UI remains as set by connected_but_unsynced, ready for user interaction

    @Slot()
    def start_robot_discovery(self):
        self.log_to_terminal("Starting robot discovery...")
        self.udp_discovery_receiver.start_listening()
        self.discover_button.setEnabled(False);
        self.discover_button.setText("Discovering...")
        QTimer.singleShot(5000, self.stop_robot_discovery_if_not_found)

    @Slot(dict)
    def handle_discovery_beacon(self, beacon_json: dict):
        self.log_to_terminal(f"Robot discovery beacon received: {beacon_json}")
        ip = beacon_json.get("ip");
        tcp_port = beacon_json.get("tcp_port");
        udp_port = beacon_json.get("udp_port")
        mjpeg_port = beacon_json.get("mjpeg_port")  # Check if beacon includes mjpeg_port

        if ip and tcp_port and udp_port:
            self.robot_ip_input.setText(ip)
            self.robot_tcp_port_input.setText(str(tcp_port))
            self.robot_udp_port_input.setText(str(udp_port))
            if mjpeg_port: self.robot_mjpeg_port_input.setText(str(mjpeg_port))
            self.log_to_terminal(f"Populated connection fields from beacon: IP={ip}, TCP={tcp_port}, UDP={udp_port}" + (
                f", MJPEG={mjpeg_port}" if mjpeg_port else ""))
        self.udp_discovery_receiver.stop_listening()
        self.discover_button.setEnabled(True);
        self.discover_button.setText("Discover Robot")

    @Slot()
    def stop_robot_discovery_if_not_found(self):
        if self.udp_discovery_receiver.running:
            self.log_to_terminal("Robot discovery timeout. No beacon received.")
            self.udp_discovery_receiver.stop_listening()
            self.discover_button.setEnabled(True);
            self.discover_button.setText("Discover Robot")

    @Slot()
    def toggle_connection(self):
        if not self.comms_client or not self.comms_client.is_tcp_connected():
            robot_ip = self.robot_ip_input.text()
            try:
                robot_tcp_port = int(self.robot_tcp_port_input.text())
                robot_udp_port = int(self.robot_udp_port_input.text())
                # MJPEG port validation could be added if strictly needed here, but it's mainly for stream URL
                if not (1024 <= robot_tcp_port <= 65535 and 1024 <= robot_udp_port <= 65535):
                    raise ValueError("Port numbers out of range.")
            except ValueError as e:
                self.log_to_terminal(f"Invalid port number(s): {e}");
                self.connection_status_label.setText("Status: Invalid Port(s)");
                self.connection_status_label.setStyleSheet("color: red");
                return

            self.connect_button.setEnabled(False);
            self.discover_button.setEnabled(False)
            self.connection_status_label.setText(f"Status: Connecting to {robot_ip}...");
            self.connection_status_label.setStyleSheet("color: orange")
            self.log_to_terminal(f"Attempting connection to {robot_ip} (TCP:{robot_tcp_port}, UDP:{robot_udp_port})")

            self.comms_client = HexapodCommsClient(robot_ip, robot_tcp_port, robot_udp_port,
                                                   DEFAULT_GUI_TELEMETRY_LISTEN_IP, DEFAULT_GUI_TELEMETRY_LISTEN_PORT)
            self.comms_client.tcp_connected_signal.connect(self.handle_tcp_conn_success)
            self.comms_client.tcp_disconnected_signal.connect(self.handle_tcp_conn_loss)
            self.comms_client.tcp_message_received_signal.connect(self.handle_tcp_message_from_esp)
            self.comms_client.rtt_updated_signal.connect(self.handle_rtt_update)
            if not self.comms_client.connect_tcp():
                # handle_tcp_conn_loss will be called via signal from CommsClient if connect_tcp fails
                # and it will re-enable the connect button.
                self.log_to_terminal("Connection attempt failed in toggle_connection.")
                # Ensure button is re-enabled if connect_tcp fails silently or before signal emission path completes fully
                # self.set_ui_for_disconnected_state() # Redundant if signal path is robust
                pass
        else:  # Disconnect
            self.log_to_terminal("Disconnecting...")
            self._stop_mjpeg_display_logic(send_stop_command=True)  # Stop stream before full disconnect
            if self.comms_client:
                self.comms_client.send_disconnect_notice();
                time.sleep(0.05)  # Give notice time to send
                self.comms_client.close();  # This calls comms_client.disconnect_tcp()
                self.comms_client = None
            self.udp_intent_timer.stop()
            self.udp_telemetry_receiver.stop_listening()
            self.set_ui_for_disconnected_state()  # This will re-enable connect button
            self.log_to_terminal("Disconnected.")

    @Slot()
    def handle_tcp_conn_success(self):
        self.log_to_terminal(f"TCP Connection successful to {self.comms_client.robot_tcp_addr[0]}.")
        self.connection_status_label.setText("Status: TCP Connected. Initializing...");
        self.connection_status_label.setStyleSheet("color: darkGreen")
        self.set_ui_for_connected_but_unsynced_state()  # This enables connect button with "Disconnect" text
        self.udp_telemetry_receiver.start_listening()
        self.send_client_settings_to_robot();
        time.sleep(0.05)  # Allow time for settings to be processed
        self.request_full_state_from_robot()
        self.update_udp_intent_timer_interval();
        self.udp_intent_timer.start()

    @Slot(str)
    def handle_tcp_conn_loss(self, reason: str):
        self.log_to_terminal(f"TCP Connection Lost/Failed: {reason}")
        self.connection_status_label.setText(f"Status: TCP Error - {reason}");
        self.connection_status_label.setStyleSheet("color: red")
        self.rtt_label.setText("RTT: N/A ms")
        self._stop_mjpeg_display_logic(send_stop_command=False)  # Stream is implicitly dead or should be stopped
        self.udp_intent_timer.stop()
        if self.comms_client:
            # CommsClient's disconnect_tcp should handle its internal cleanup.
            # We just need to nullify our reference and stop GUI-side things.
            # self.comms_client.close() # This is likely already called or will be if disconnect_tcp path is robust
            self.comms_client = None  # Nullify the reference

        self.udp_telemetry_receiver.stop_listening()
        self.set_ui_for_disconnected_state()  # This re-enables connect button with "Connect" text and discover

    @Slot(float)
    def handle_rtt_update(self, rtt_ms: float):
        self.rtt_label.setText(f"RTT: {rtt_ms:.0f} ms")

    @Slot(dict)
    def handle_tcp_message_from_esp(self, message_json: dict):
        # Potentially large messages, log only type or summary for some
        msg_type = message_json.get("type");
        payload = message_json.get("payload")
        if msg_type != "pong":  # Avoid logging frequent pongs if any slip through
            self.log_to_terminal(
                f"RX TCP: Type='{msg_type}', PayloadKeys={list(payload.keys()) if payload else 'None'}")

        if msg_type == "full_state_response" and payload:
            self.populate_gui_from_full_state(payload)
            self.connection_status_label.setText("Status: Connected & Synced");
            self.connection_status_label.setStyleSheet("color: green")
            self.set_ui_for_synced_state()
            self.log_to_terminal("Robot state synchronized with GUI.")
        elif msg_type == "telemetry_data" and payload:
            if "battery" in payload:
                v = payload["battery"].get("voltage_v", "N/A");
                p = payload["battery"].get("percentage", "")
                self.actual_battery_voltage_label.setText(
                    f"{v:.2f}V ({p}%)" if isinstance(v, float) and p else f"{v:.2f}V" if isinstance(v,
                                                                                                    float) else f"{v}V")
            if "robot_status" in payload:
                rssi = payload["robot_status"].get("wifi_rssi_dbm", "N/A");
                ctrl = payload["robot_status"].get("active_controller_hint", "N/A")
                self.actual_robot_status_label.setText(f"RSSI:{rssi}, Ctrl:{ctrl}")
        elif msg_type == "camera_stream_ack" and payload:
            action = payload.get("action_requested");
            success = payload.get("success", False)
            if action == "start" and success:
                self.log_to_terminal("ESP32 ACKed camera stream start. Initiating display.")
                self._start_mjpeg_display_logic()  # This will switch UI to stream view
            elif action == "start" and not success:
                self.log_to_terminal(f"ESP32 NACKed camera stream start: {payload.get('message', 'No details')}")
                self.video_start_stream_button.setEnabled(True);
                self.video_apply_config_button.setEnabled(True)  # Re-enable
            elif action == "stop" and success:
                self.log_to_terminal("ESP32 ACKed camera stream stop. UI should reflect this.")
                # _stop_mjpeg_display_logic was called by button, which updates UI. This ACK confirms.
                # on_stream_thread_finished (called by worker finishing) will ensure UI is on config view
            elif action == "stop" and not success:
                self.log_to_terminal(f"ESP32 NACKed camera stream stop: {payload.get('message', 'No details')}")
                # If stop failed on ESP, stream might still be running there.
                # Local UI should still be reset by _stop_mjpeg_display_logic
                self.on_stream_thread_finished()  # Ensure UI is reset to config state
        elif msg_type == "camera_config_ack" and payload:
            success = payload.get("success", False)
            if success:
                self.log_to_terminal(
                    f"ESP32 ACKed camera config update. Current ESP config: {payload.get('current_config', {})}")
            else:
                self.log_to_terminal(f"ESP32 NACKed camera config update: {payload.get('message', 'No details')}")
            self.video_apply_config_button.setEnabled(True)  # Re-enable

    def populate_gui_from_full_state(self, payload: dict):
        for sb in self.all_config_spinboxes: sb.blockSignals(True)
        if "max_speeds" in payload:
            self.cfg_max_linear_speed_input.setValue(payload["max_speeds"].get("linear_cms", GUI_REF_MAX_LINEAR_SPEED))
            self.cfg_max_yaw_rate_input.setValue(payload["max_speeds"].get("yaw_rads", GUI_REF_MAX_YAW_RATE))
        if "pose_adjust_speeds" in payload:
            self.cfg_pose_linear_speed_input.setValue(
                payload["pose_adjust_speeds"].get("linear_cms", GUI_REF_POSE_ADJUST_SPEED_LINEAR))
            self.cfg_pose_angular_speed_input.setValue(
                math.degrees(payload["pose_adjust_speeds"].get("angular_rads", GUI_REF_POSE_ADJUST_SPEED_ANGULAR)))
        if "acceleration_values" in payload:
            accel_data = payload["acceleration_values"]
            self.cfg_linear_accel_input.setValue(accel_data.get("linear_accel_cms2", GUI_REF_LINEAR_ACCEL_CMS2))
            self.cfg_linear_decel_input.setValue(accel_data.get("linear_decel_cms2", GUI_REF_LINEAR_DECEL_CMS2))
            self.cfg_yaw_accel_input.setValue(
                math.degrees(accel_data.get("yaw_accel_rads2", math.radians(GUI_REF_YAW_ACCEL_DEGS2))))
            self.cfg_yaw_decel_input.setValue(
                math.degrees(accel_data.get("yaw_decel_rads2", math.radians(GUI_REF_YAW_DECEL_DEGS2))))
        if "gait_params" in payload:
            self.cfg_step_height_input.setValue(payload["gait_params"].get("step_height_cm", GUI_REF_STEP_HEIGHT_CM))
            self.cfg_step_time_input.setValue(payload["gait_params"].get("step_time_s", GUI_REF_STEP_TIME_S))
        if "leg_geometry_abstract" in payload:
            geom_data = payload["leg_geometry_abstract"]
            self.cfg_leg_front_corner_x.setValue(geom_data.get("front_corner_x_cm", DEFAULT_LEG_POS_FR_X_CM))
            self.cfg_leg_front_corner_y.setValue(geom_data.get("front_corner_y_cm", DEFAULT_LEG_POS_FR_Y_CM))
            self.cfg_leg_middle_side_x.setValue(geom_data.get("middle_side_x_cm", DEFAULT_LEG_POS_MR_X_CM))
            self.cfg_leg_corner_ext.setValue(geom_data.get("corner_ext_cm", DEFAULT_LEG_CORNER_EXT_CM))
            self.cfg_leg_middle_ext.setValue(geom_data.get("middle_ext_cm", DEFAULT_LEG_MIDDLE_EXT_CM))
        if "current_body_pose" in payload:  # Assuming this part of payload for actuals
            pose = payload["current_body_pose"]
            if "position_offset_cm" in pose:
                self.actual_pos_x_label.setText(f"{pose['position_offset_cm'].get('x', 0):.2f} cm")
                self.actual_pos_y_label.setText(f"{pose['position_offset_cm'].get('y', 0):.2f} cm")
                self.actual_pos_z_label.setText(f"{pose['position_offset_cm'].get('z', 0):.2f} cm")
            if "orientation_quat" in pose:
                self.actual_orient_w_label.setText(f"{pose['orientation_quat'].get('w', 1):.3f}")
                self.actual_orient_x_label.setText(f"{pose['orientation_quat'].get('x', 0):.3f}")
                self.actual_orient_y_label.setText(f"{pose['orientation_quat'].get('y', 0):.3f}")
                self.actual_orient_z_label.setText(f"{pose['orientation_quat'].get('z', 0):.3f}")
        self.update_walk_active_display(payload.get("walk_active", False))
        # Camera config from full state (if ESP provides it)
        if "camera_current_config" in payload:
            cam_cfg = payload["camera_current_config"]
            self.video_resolution_combo.setCurrentText(cam_cfg.get("resolution", FRAMESIZE_STR_QVGA))
            self.video_quality_spin.setValue(cam_cfg.get("quality", 12))
            self.video_fps_spin.setValue(cam_cfg.get("fps_limit", 10))
            self.log_to_terminal("Populated camera config from robot state.")

        for sb in self.all_config_spinboxes: sb.blockSignals(False)
        self.update_gui_calculated_base_pos_display()

    def update_walk_active_display(self, is_active: bool):
        self.actual_walk_status_label.setText("ACTIVE" if is_active else "NOT ACTIVE")
        self.actual_walk_status_label.setStyleSheet("color: green" if is_active else "color: red")

    @Slot(dict)
    def handle_udp_telemetry(self, telemetry_json: dict):
        payload = telemetry_json.get("payload", {});
        msg_type = telemetry_json.get("type", "")
        if msg_type == "robot_state_telemetry":
            if "locomotion_actual" in payload:
                loco = payload["locomotion_actual"]
                self.actual_velocity_x_label.setText(f"{loco.get('velocity_x_cms', 0):.2f} cm/s")
                self.actual_velocity_y_label.setText(f"{loco.get('velocity_y_cms', 0):.2f} cm/s")
                self.actual_yaw_rate_label.setText(f"{loco.get('angular_velocity_yaw_rads', 0):.3f} rad/s")
            if "body_pose_actual" in payload:
                pose = payload["body_pose_actual"]
                if "position_offset_cm" in pose:
                    pos = pose["position_offset_cm"]
                    self.actual_pos_x_label.setText(f"{pos.get('x', 0):.2f} cm");
                    self.actual_pos_y_label.setText(f"{pos.get('y', 0):.2f} cm");
                    self.actual_pos_z_label.setText(f"{pos.get('z', 0):.2f} cm")
                if "orientation_quat" in pose:
                    orient = pose["orientation_quat"]
                    self.actual_orient_w_label.setText(f"{orient.get('w', 1):.3f}");
                    self.actual_orient_x_label.setText(f"{orient.get('x', 0):.3f}");
                    self.actual_orient_y_label.setText(f"{orient.get('y', 0):.3f}");
                    self.actual_orient_z_label.setText(f"{orient.get('z', 0):.3f}")
            if "gait_actual" in payload:
                gait = payload["gait_actual"]
                self.actual_gait_h_label.setText(f"{gait.get('step_height_cm', 0):.1f} cm");
                self.actual_gait_t_label.setText(f"{gait.get('step_time_s', 0):.1f} s")
                self.update_walk_active_display(gait.get('walk_active', False))
            if "debug_foot_pos_walk_cm" in payload:
                fp_walk_data = payload["debug_foot_pos_walk_cm"]
                if isinstance(fp_walk_data, list) and self.debug_foot_pos_group:
                    self.debug_foot_pos_group.setVisible(self.sub_debug_foot_pos_check.isChecked())
                    for i in range(min(len(fp_walk_data), LEG_COUNT)):
                        leg_data = fp_walk_data[i];
                        x, y, z = leg_data.get('x', 'N/A'), leg_data.get('y', 'N/A'), leg_data.get('z', 'N/A')
                        self.debug_fp_walk_labels[i]['x'].setText(f"X:{x:.1f}" if isinstance(x, float) else f"X:{x}")
                        self.debug_fp_walk_labels[i]['y'].setText(f"Y:{y:.1f}" if isinstance(y, float) else f"Y:{y}")
                        self.debug_fp_walk_labels[i]['z'].setText(f"Z:{z:.1f}" if isinstance(z, float) else f"Z:{z}")
            elif self.debug_foot_pos_group and not self.sub_debug_foot_pos_check.isChecked():
                self.debug_foot_pos_group.setVisible(False)

    @Slot()
    def update_udp_intent_timer_interval(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            try:
                freq_hz = self.update_freq_input.value()
                self.udp_intent_timer.setInterval(
                    int(1000.0 / max(MIN_UPDATE_FREQUENCY_HZ, min(freq_hz, MAX_UPDATE_FREQUENCY_HZ))))
            except Exception:
                self.udp_intent_timer.setInterval(int(1000.0 / DEFAULT_UPDATE_FREQUENCY_HZ))

    def keyPressEvent(self, event: QKeyEvent):
        if not self.comms_client or not self.comms_client.is_tcp_connected(): return
        if not event.isAutoRepeat():
            self.keys_pressed.add(event.key());
            self.update_intent_factors_from_keys()
            if event.key() == Qt.Key.Key_Space:
                robot_is_walking = (self.actual_walk_status_label.text() == "ACTIVE")
                new_desired_state = not robot_is_walking
                if self.comms_client.send_gait_command(new_desired_state):
                    self.log_to_terminal(f"TX TCP: GaitCommand walk_active: {new_desired_state}")

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            try:
                self.keys_pressed.remove(event.key())
            except KeyError:
                pass
            self.update_intent_factors_from_keys()

    def update_intent_factors_from_keys(self):
        self.loco_intent_vy_factor = (1.0 if Qt.Key.Key_W in self.keys_pressed else 0.0) + (
            -1.0 if Qt.Key.Key_S in self.keys_pressed else 0.0)
        self.loco_intent_vx_factor = (-1.0 if Qt.Key.Key_A in self.keys_pressed else 0.0) + (
            1.0 if Qt.Key.Key_D in self.keys_pressed else 0.0)
        self.loco_intent_yaw_factor = (1.0 if Qt.Key.Key_E in self.keys_pressed else 0.0) + (
            -1.0 if Qt.Key.Key_Q in self.keys_pressed else 0.0)
        shift_pressed = bool(QApplication.keyboardModifiers() & Qt.KeyboardModifier.ShiftModifier)
        self.pose_adjust_intent_offset_y = 0.0;
        self.pose_adjust_intent_offset_x = 0.0;
        self.pose_adjust_intent_offset_z = 0.0
        if Qt.Key.Key_Up in self.keys_pressed: self.pose_adjust_intent_offset_z = 1.0 if shift_pressed else 0.0; self.pose_adjust_intent_offset_y = 1.0 if not shift_pressed else 0.0
        if Qt.Key.Key_Down in self.keys_pressed: self.pose_adjust_intent_offset_z = -1.0 if shift_pressed else 0.0; self.pose_adjust_intent_offset_y = -1.0 if not shift_pressed else 0.0
        if Qt.Key.Key_Left in self.keys_pressed and not shift_pressed: self.pose_adjust_intent_offset_x = -1.0
        if Qt.Key.Key_Right in self.keys_pressed and not shift_pressed: self.pose_adjust_intent_offset_x = 1.0
        self.pose_adjust_intent_pitch = (1.0 if Qt.Key.Key_K in self.keys_pressed else 0.0) + (
            -1.0 if Qt.Key.Key_I in self.keys_pressed else 0.0)
        self.pose_adjust_intent_roll = (-1.0 if Qt.Key.Key_J in self.keys_pressed else 0.0) + (
            1.0 if Qt.Key.Key_L in self.keys_pressed else 0.0)
        self.pose_adjust_intent_body_yaw = (-1.0 if Qt.Key.Key_O in self.keys_pressed else 0.0) + (
            1.0 if Qt.Key.Key_U in self.keys_pressed else 0.0)
        self.centering_xy_active = Qt.Key.Key_C in self.keys_pressed
        self.centering_orientation_active = Qt.Key.Key_X in self.keys_pressed

    @Slot()
    def send_active_intents_udp(self):
        current_time = time.time()
        self.log_to_terminal(f"DEBUG: send_active_intents_udp called at {current_time:.3f}")  # Log when method is called
        if not self.comms_client or not self.comms_client.is_tcp_connected(): return
        self.comms_client.send_locomotion_intent(self.loco_intent_vx_factor, self.loco_intent_vy_factor,
                                                 self.loco_intent_yaw_factor)
        self.comms_client.send_pose_adjust_intent(self.pose_adjust_intent_offset_x, self.pose_adjust_intent_offset_y,
                                                  self.pose_adjust_intent_offset_z, self.pose_adjust_intent_pitch,
                                                  self.pose_adjust_intent_roll, self.pose_adjust_intent_body_yaw)
        self.comms_client.send_centering_intent(self.centering_xy_active, self.centering_orientation_active)

    @Slot()
    def send_movement_configs_auto(self):
        if self.comms_client and self.comms_client.is_tcp_connected() and (
                not self.sender() or self.sender().isEnabled()):
            max_s = {"linear_cms": self.cfg_max_linear_speed_input.value(),
                     "yaw_rads": self.cfg_max_yaw_rate_input.value()}
            pose_s = {"linear_cms": self.cfg_pose_linear_speed_input.value(),
                      "angular_rads": math.radians(self.cfg_pose_angular_speed_input.value())}
            if self.comms_client.send_config_update(max_speeds=max_s, pose_adjust_speeds=pose_s): self.log_to_terminal(
                f"TX TCP Config: Speeds & Pose Rates Update")

    @Slot()
    def send_accel_configs_auto(self):
        if self.comms_client and self.comms_client.is_tcp_connected() and (
                not self.sender() or self.sender().isEnabled()):
            accel_p = {"linear_accel_cms2": self.cfg_linear_accel_input.value(),
                       "linear_decel_cms2": self.cfg_linear_decel_input.value(),
                       "yaw_accel_rads2": math.radians(self.cfg_yaw_accel_input.value()),
                       "yaw_decel_rads2": math.radians(self.cfg_yaw_decel_input.value())}
            if self.comms_client.send_config_update(acceleration_values=accel_p): self.log_to_terminal(
                f"TX TCP Config: Acceleration Update")

    @Slot()
    def send_gait_configs_auto(self):
        if self.comms_client and self.comms_client.is_tcp_connected() and (
                not self.sender() or self.sender().isEnabled()):
            gait_p = {"step_height_cm": self.cfg_step_height_input.value(),
                      "step_time_s": self.cfg_step_time_input.value()}
            if self.comms_client.send_config_update(gait_params=gait_p): self.log_to_terminal(
                f"TX TCP Config: Gait Update")

    @Slot()
    def send_leg_geometry_configs_auto(self):
        self.update_gui_calculated_base_pos_display()
        if self.comms_client and self.comms_client.is_tcp_connected() and (
                not self.sender() or self.sender().isEnabled()):
            geom_p = {"front_corner_x_cm": self.cfg_leg_front_corner_x.value(),
                      "front_corner_y_cm": self.cfg_leg_front_corner_y.value(),
                      "middle_side_x_cm": self.cfg_leg_middle_side_x.value(),
                      "corner_ext_cm": self.cfg_leg_corner_ext.value(),
                      "middle_ext_cm": self.cfg_leg_middle_ext.value()}
            if self.comms_client.send_config_update(leg_geometry_abstract=geom_p): self.log_to_terminal(
                f"TX TCP Config: Leg Geometry Update")

    def update_gui_calculated_base_pos_display(self):
        self.calculated_base_positions = []
        fr_x = self.cfg_leg_front_corner_x.value();
        fr_y = self.cfg_leg_front_corner_y.value();
        mr_x = self.cfg_leg_middle_side_x.value()
        corner_ext = self.cfg_leg_corner_ext.value();
        middle_ext = self.cfg_leg_middle_ext.value()
        sym_base_xy = [(fr_x, -fr_y), (mr_x, 0.0), (fr_x, fr_y), (-fr_x, -fr_y), (-mr_x, 0.0), (-fr_x, fr_y)]
        for i in range(LEG_COUNT):
            base_x, base_y = sym_base_xy[i];
            is_middle = (LEG_NAMES[i] in ["MR", "ML"]);
            ext = middle_ext if is_middle else corner_ext
            angle = ESP_LEG_MOUNTING_ANGLES[i];
            final_x = base_x + ext * math.cos(angle);
            final_y = base_y + ext * math.sin(angle)
            self.calculated_base_positions.append({'x': final_x, 'y': final_y, 'z': 0.0})
            self.gui_base_pos_labels[i]['x'].setText(f"X:{final_x:.1f}");
            self.gui_base_pos_labels[i]['y'].setText(f"Y:{final_y:.1f}");
            self.gui_base_pos_labels[i]['z'].setText(f"Z:0.0")

    @Slot()
    def send_pwm_reset_command_tcp(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            if self.comms_client.send_pwm_reinitialize_command(): self.log_to_terminal(
                "TX TCP: Sent PWM Re-initialize Command.")

    @Slot()
    def send_client_settings_to_robot(self):
        if not self.comms_client or not self.comms_client.is_tcp_connected(): self.log_to_terminal(
            "Cannot send client_settings: Not connected."); return
        try:
            gui_ip = self.comms_client.get_local_ip_for_telemetry_setup()
            if not gui_ip or gui_ip == "0.0.0.0": self.log_to_terminal(
                "[ERROR] Could not determine valid local IP for telemetry. Cannot send client_settings."); return
            udp_subs = {"robot_state_actual": {"enabled": self.sub_robot_state_actual_check.isChecked(),
                                               "interval_ms": int(self.sub_robot_state_actual_interval.text())},
                        "debug_foot_pos": {"enabled": self.sub_debug_foot_pos_check.isChecked(),
                                           "interval_ms": int(self.sub_debug_foot_pos_interval.text())}}
            tcp_subs = {"battery": {"enabled": self.sub_battery_check.isChecked(),
                                    "interval_ms": int(self.sub_battery_interval.text())},
                        "robot_status": {"enabled": self.sub_robot_status_check.isChecked(),
                                         "interval_ms": int(self.sub_robot_status_interval.text())}}
            settings_p = {
                "udp_telemetry_config": {"target_ip": gui_ip, "target_port": DEFAULT_GUI_TELEMETRY_LISTEN_PORT,
                                         "subscriptions": udp_subs}, "tcp_subscriptions_here": tcp_subs}
            if self.comms_client.send_client_settings(settings_p): self.log_to_terminal(
                f"TX TCP: Sent client_settings (UDP telemetry to {gui_ip}:{DEFAULT_GUI_TELEMETRY_LISTEN_PORT}).")
            self.debug_foot_pos_group.setVisible(self.sub_debug_foot_pos_check.isChecked())
        except ValueError:
            self.log_to_terminal("[ERROR] Invalid interval for telemetry subscriptions.")
        except Exception as e:
            self.log_to_terminal(f"[ERROR] Sending client_settings: {e}")

    @Slot()
    def request_full_state_from_robot(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            gui_ip = self.comms_client.get_local_ip_for_telemetry_setup()
            if not gui_ip or gui_ip == "0.0.0.0": self.log_to_terminal(
                "[ERROR] Could not determine valid local IP for telemetry. Cannot request full state."); return
            if self.comms_client.send_request_full_state(gui_ip): self.log_to_terminal(
                f"TX TCP: Requesting full state (reply to {gui_ip})")
        else:
            self.log_to_terminal("Cannot request full state: Not connected via TCP.")

    # --- Camera Control Slots ---
    @Slot()
    def send_camera_start_stream_command_tcp(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            self.log_to_terminal("TX TCP: Requesting camera stream START.")
            # Disable buttons until ACK or error
            self.video_start_stream_button.setEnabled(False)
            self.video_apply_config_button.setEnabled(False)
            if not self.comms_client.send_camera_stream_control("start"):
                self.log_to_terminal(
                    "Failed to send camera stream start command (send_camera_stream_control returned False).")
                # Re-enable buttons if send failed immediately (e.g. TCP disconnected during send)
                if self.comms_client and self.comms_client.is_tcp_connected():  # Check if still connected
                    self.video_start_stream_button.setEnabled(True);
                    self.video_apply_config_button.setEnabled(True)
                else:  # Disconnected, UI should reflect this broadly
                    self.on_stream_thread_finished()  # Reset camera UI part
        else:
            self.log_to_terminal("Cannot start camera stream: Not connected.")

    @Slot()
    def send_camera_stop_stream_command_tcp(self):
        self.log_to_terminal("GUI: User requested STOP stream.")
        self._stop_mjpeg_display_logic(send_stop_command=True)

    @Slot()
    def send_camera_config_command_tcp(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            res = self.video_resolution_combo.currentText()
            qual = self.video_quality_spin.value()
            fps = self.video_fps_spin.value()
            self.log_to_terminal(f"TX TCP: Sending camera config: Res={res}, Q={qual}, FPS={fps}")
            self.video_apply_config_button.setEnabled(False)  # Disable until ACK
            if not self.comms_client.send_camera_config_update(res, qual, fps):
                self.log_to_terminal(
                    "Failed to send camera config update command (send_camera_config_update returned False).")
                if self.comms_client and self.comms_client.is_tcp_connected():
                    self.video_apply_config_button.setEnabled(True)  # Re-enable if still connected
                # If disconnected, broad UI update will occur
        else:
            self.log_to_terminal("Cannot send camera config: Not connected.")

    def _start_mjpeg_display_logic(self):
        if self._stream_thread and self._stream_thread.isRunning():
            self.log_to_terminal("MJPEG display thread already running or starting.")
            return

        robot_ip = self.robot_ip_input.text()
        try:
            mjpeg_port = int(self.robot_mjpeg_port_input.text())
        except ValueError:
            self.log_to_terminal("Error: Invalid MJPEG Stream Port."); return

        stream_url = f"http://{robot_ip}:{mjpeg_port}/stream"
        self.log_to_terminal(f"Initializing MJPEG display thread for URL: {stream_url}")

        self._stream_thread = QThread(self)
        self._stream_thread.setObjectName("MjpegStreamThread")  # For debugging
        self._stream_worker = MjpegStreamWorker()
        self._stream_worker.moveToThread(self._stream_thread)
        self._stream_worker.set_url(stream_url)

        self._stream_worker.new_frame_signal.connect(self.update_video_frame)
        self._stream_worker.stream_error_signal.connect(self.handle_stream_error)

        self._stream_worker.finished.connect(self._stream_thread.quit)
        # Ensure deleteLater is called for both worker and thread when thread is finished
        self._stream_thread.finished.connect(self._stream_worker.deleteLater)
        self._stream_thread.finished.connect(self._stream_thread.deleteLater)
        self._stream_thread.finished.connect(self.on_stream_thread_finished)  # GUI cleanup slot

        self._stream_thread.started.connect(self._stream_worker.start_stream)
        self._stream_thread.start()
        self.video_label.setText("Connecting to MJPEG stream...");
        self.video_label.setStyleSheet("background-color: black; color: yellow;")
        self.video_stacked_widget.setCurrentIndex(1)  # Switch to stream view

    def _stop_mjpeg_display_logic(self, send_stop_command=True):
        self.log_to_terminal(f"GUI: _stop_mjpeg_display_logic called (send_stop_command={send_stop_command})")

        if send_stop_command and self.comms_client and self.comms_client.is_tcp_connected():
            self.log_to_terminal("TX TCP: Requesting camera stream STOP via _stop_mjpeg_display_logic.")
            if not self.comms_client.send_camera_stream_control("stop"):
                self.log_to_terminal("Failed to send camera stream stop command to ESP32.")

        # Signal worker to stop its processing loop
        if self._stream_worker:
            self.log_to_terminal("Signaling MJPEG worker to stop its loop...")
            self._stream_worker.stop_stream()  # This sets the worker's _is_running to False

        # Wait for the thread to finish gracefully
        if self._stream_thread and self._stream_thread.isRunning():
            self.log_to_terminal("Waiting for MJPEG display thread to quit (max 2s)...")
            if not self._stream_thread.wait(2000):  # Wait up to 2 seconds
                self.log_to_terminal("Warning: MJPEG display thread did not quit gracefully within timeout.")
                # Do NOT terminate. Rely on worker's finally block and finished signal.
            else:
                self.log_to_terminal("MJPEG display thread quit gracefully.")
        else:
            self.log_to_terminal("MJPEG display thread not running or already gone.")
            # If thread wasn't running, but worker/thread objects might exist,
            # ensure UI cleanup is triggered if it hasn't been by the thread.finished signal
            if self._stream_thread is not None or self._stream_worker is not None:
                self.on_stream_thread_finished()  # Manually trigger UI cleanup

        # The on_stream_thread_finished slot (connected to thread.finished) will handle
        # nullifying _stream_thread, _stream_worker, and resetting UI elements.

    @Slot(QImage)
    def update_video_frame(self, q_image: QImage):
        if not q_image.isNull() and self.video_label.isVisible():  # Check if label is visible
            # Use FastTransformation for potentially better performance with laggy streams
            pixmap = QPixmap.fromImage(q_image)
            self.video_label.setPixmap(pixmap.scaled(
                self.video_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation  # Changed to FastTransformation
            ))
        # else: self.log_to_terminal("Received null QImage or label not visible.")

    @Slot(str)
    def handle_stream_error(self, error_msg: str):
        self.log_to_terminal(f"Stream Display Error: {error_msg}")
        self.video_label.setText(f"Stream Error:\n{error_msg}");
        self.video_label.setStyleSheet("background-color: darkred; color: white;")
        # Worker's finished signal will trigger on_stream_thread_finished for full cleanup.
        # This slot is just for updating the error message.
        # However, if the error means the stream is dead, ensure the UI reflects this sooner.
        if self.video_stacked_widget.currentIndex() == 1:  # If we were in stream view
            self.on_stream_thread_finished()  # Force UI reset to config state

    @Slot()
    def on_stream_thread_finished(self):
        # This slot is connected to self._stream_thread.finished
        self.log_to_terminal("GUI: on_stream_thread_finished triggered.")

        # Nullify references to allow QObject's deleteLater to work without dangling Python refs
        self._stream_thread = None
        self._stream_worker = None

        # Reset UI elements related to the stream
        if self.video_stacked_widget.currentIndex() == 1:  # If we were in stream view
            self.video_stacked_widget.setCurrentIndex(0)  # Switch back to config

        # Re-enable config/start buttons only if TCP is still connected
        if self.comms_client and self.comms_client.is_tcp_connected():
            self.video_start_stream_button.setEnabled(True)
            self.video_apply_config_button.setEnabled(True)
            self.video_resolution_combo.setEnabled(True)
            self.video_quality_spin.setEnabled(True)
            self.video_fps_spin.setEnabled(True)
        else:  # If TCP is not connected, keep them disabled
            self.video_start_stream_button.setEnabled(False)
            self.video_apply_config_button.setEnabled(False)
            self.video_resolution_combo.setEnabled(False)
            self.video_quality_spin.setEnabled(False)
            self.video_fps_spin.setEnabled(False)

        self.video_label.setText("Stream stopped/offline.");
        self.video_label.setStyleSheet("background-color: black; color: gray;")
        self.log_to_terminal("GUI: MJPEG stream UI reset and references cleared.")

    def closeEvent(self, event: QCloseEvent):
        self.log_to_terminal("Close event: Shutting down...")

        # Attempt to stop ESP32 stream if connected
        if self.comms_client and self.comms_client.is_tcp_connected():
            self.log_to_terminal("CloseEvent: Sending camera stream stop command.")
            self.comms_client.send_camera_stream_control("stop")
            time.sleep(0.05)  # Give it a moment
            self.log_to_terminal("CloseEvent: Sending disconnect notice.")
            self.comms_client.send_disconnect_notice()
            time.sleep(0.1)  # Give it a moment

        # Stop local MJPEG display thread
        self._stop_mjpeg_display_logic(send_stop_command=False)  # Don't resend stop command to ESP

        self.udp_intent_timer.stop()
        self.udp_telemetry_receiver.stop_listening()
        self.udp_discovery_receiver.stop_listening()

        if self.comms_client:
            self.comms_client.close();  # This calls disconnect_tcp internally
            self.comms_client = None

        self.log_to_terminal("Shutdown procedures complete.")
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HexapodControllerGUI()
    window.show()
    sys.exit(app.exec())