#!/usr/bin/env python3
import sys
import math
import time
import json
import socket
import threading

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QGroupBox, QFormLayout,
    QSizePolicy, QCheckBox, QScrollArea
)
from PySide6.QtCore import Qt, QTimer, Slot, QElapsedTimer, Signal, QObject
from PySide6.QtGui import QKeyEvent, QCloseEvent, QDoubleValidator

try:
    from hexapod_udp_client import HexapodUDPClient  # NEW JSON Client

    LEG_COUNT = 6
except ImportError:
    sys.exit("Error: Could not find hexapod_udp_client.py.")

# --- Configuration ---
DEFAULT_ROBOT_TARGET_IP = "255.255.255.255"
DEFAULT_ROBOT_TARGET_PORT = 5005
DEFAULT_GUI_TELEMETRY_LISTEN_IP = "0.0.0.0"
DEFAULT_GUI_TELEMETRY_LISTEN_PORT = 5007
DEFAULT_UPDATE_FREQUENCY_HZ = 20.0
MIN_UPDATE_FREQUENCY_HZ = 5.0
MAX_UPDATE_FREQUENCY_HZ = 50.0

GUI_REF_MAX_LINEAR_SPEED = 10.0
GUI_REF_MAX_YAW_RATE = 0.5
GUI_REF_POSE_ADJUST_SPEED_LINEAR = 2.0
GUI_REF_POSE_ADJUST_SPEED_ANGULAR = math.radians(15.0)

LEG_NAMES = ["BR", "MR", "FR", "BL", "ML", "FL"]
DEFAULT_BASE_POSITIONS_FOR_GUI_CALC = [
    [27.0, -19.0, 0.0], [32.0, 0.0, 0.0], [27.0, 19.0, 0.0],
    [-27.0, -19.0, 0.0], [-32.0, 0.0, 0.0], [-27.0, 19.0, 0.0]
]
LEG_MOUNTING_ANGLES_FOR_GUI_CALC = [
    -1 * math.pi / 4.0, 0 * math.pi / 4.0, +1 * math.pi / 4.0, -3 * math.pi / 4.0, +4 * math.pi / 4.0,
    +3 * math.pi / 4.0
]


class Quaternion:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w, self.x, self.y, self.z = w, x, y, z; self.normalize()

    def normalize(self):
        mag_sq = self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2
        if mag_sq > 1e-12:  # Check against squared epsilon
            mag = math.sqrt(mag_sq)
            self.w /= mag;
            self.x /= mag;
            self.y /= mag;
            self.z /= mag
        else:
            self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0
        return self

    def __mul__(self, o):
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z; w2, x2, y2, z2 = o.w, o.x, o.y, o.z; return Quaternion(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)

    @classmethod
    def from_axis_angle(cls, ax, ay, az, ang):
        ha = ang / 2.0;
        s = math.sin(ha);
        axis_mag_sq = ax * ax + ay * ay + az * az
        if axis_mag_sq < 1e-12: return cls()  # Return identity if axis is zero
        axis_mag = math.sqrt(axis_mag_sq)
        return cls(math.cos(ha), ax / axis_mag * s, ay / axis_mag * s, az / axis_mag * s)

    def to_dict(self):
        return {"w": self.w, "x": self.x, "y": self.y, "z": self.z}


class TelemetryReceiver(QObject):
    telemetry_received_signal = Signal(dict)

    def __init__(self, listen_ip, listen_port):
        super().__init__(); self.listen_ip, self.listen_port, self.running, self.sock = listen_ip, listen_port, False, None

    def start_listening(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.listen_ip, self.listen_port)); self.sock.settimeout(1.0); self.running = True; print(
                f"Telemetry listener on {self.listen_ip}:{self.listen_port}")
        except Exception as e:
            print(f"[ERROR] Telemetry listener: {e}"); self.running = False; return
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
                if data:
                    try:
                        json_data = json.loads(data.decode('utf-8')); self.telemetry_received_signal.emit(json_data)
                    except Exception as e:
                        print(f"[WARN] Telemetry processing error: {e}, Data: {data[:100]}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running: print(f"[ERROR] Telemetry socket: {e}"); break
        if self.sock: self.sock.close(); print("Telemetry listener stopped.")

    def stop_listening(self):
        self.running = False


class HexapodControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hexapod Intent/Config Controller")
        self.setGeometry(100, 100, 950, 800)  # Adjusted size

        self.hexapod_client = None
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.send_active_intents)

        self.telemetry_receiver = None
        self.telemetry_thread = None

        self.actual_battery_voltage_label = QLabel("N/A V")
        self.actual_robot_status_label = QLabel("N/A")
        self.actual_velocity_x_label = QLabel("0.00 cm/s")
        self.actual_velocity_y_label = QLabel("0.00 cm/s")
        self.actual_yaw_rate_label = QLabel("0.00 rad/s")
        self.actual_pos_x_label = QLabel("0.00 cm")
        self.actual_pos_y_label = QLabel("0.00 cm")
        self.actual_pos_z_label = QLabel("0.00 cm")
        self.actual_orient_w_label = QLabel("1.000")
        self.actual_orient_x_label = QLabel("0.000")
        self.actual_orient_y_label = QLabel("0.000")
        self.actual_orient_z_label = QLabel("0.000")
        self.actual_gait_h_label = QLabel("0.0 cm")
        self.actual_gait_t_label = QLabel("0.0 s")
        self.actual_walk_status_label = QLabel("NOT ACTIVE")

        self.debug_foot_pos_group = None  # QGroupBox
        self.debug_fp_walk_labels = []  # List of dicts {'x': QLabel, 'y': QLabel, 'z': QLabel}
        self.sub_debug_foot_pos_check = None  # QCheckBox
        self.sub_debug_foot_pos_interval = None  # QLineEdit

        self.keys_pressed = set()
        self.loco_intent_vx_factor = 0.0
        self.loco_intent_vy_factor = 0.0
        self.loco_intent_yaw_factor = 0.0
        self.pose_adjust_intent_offset_x = 0.0
        self.pose_adjust_intent_offset_y = 0.0
        self.pose_adjust_intent_offset_z = 0.0
        self.pose_adjust_intent_pitch = 0.0
        self.pose_adjust_intent_roll = 0.0
        self.pose_adjust_intent_body_yaw = 0.0
        self.centering_xy_active = False
        self.centering_orientation_active = False
        self.walk_command_active = False

        self._init_ui()

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        scroll_area = QScrollArea();
        scroll_area.setWidgetResizable(True)
        scroll_widget = QWidget();
        scroll_area.setWidget(scroll_widget)
        main_scroll_layout = QVBoxLayout(scroll_widget)

        connection_group = QGroupBox("Connection & Rate")
        connection_layout = QHBoxLayout(connection_group)
        connection_layout.addWidget(QLabel("Robot IP:"))
        self.robot_ip_input = QLineEdit(DEFAULT_ROBOT_TARGET_IP)
        connection_layout.addWidget(self.robot_ip_input)
        connection_layout.addWidget(QLabel("Cmd Port:"))
        self.robot_port_input = QLineEdit(str(DEFAULT_ROBOT_TARGET_PORT))
        self.robot_port_input.setFixedWidth(60);
        self.robot_port_input.setValidator(QDoubleValidator(1024, 65535, 0))
        connection_layout.addWidget(self.robot_port_input)
        connection_layout.addWidget(QLabel("Intent Send Freq (Hz):"))
        self.update_freq_input = QDoubleSpinBox()
        self.update_freq_input.setRange(MIN_UPDATE_FREQUENCY_HZ, MAX_UPDATE_FREQUENCY_HZ)
        self.update_freq_input.setValue(DEFAULT_UPDATE_FREQUENCY_HZ)
        self.update_freq_input.setDecimals(1);
        self.update_freq_input.setSingleStep(1.0)
        self.update_freq_input.valueChanged.connect(self.update_timer_interval_ui_action)  # Connect to UI action
        connection_layout.addWidget(self.update_freq_input)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_button)
        self.connection_status_label = QLabel("Status: Disconnected")
        connection_layout.addWidget(self.connection_status_label)
        main_scroll_layout.addWidget(connection_group)

        control_display_layout = QHBoxLayout()

        left_column_widget = QWidget()
        left_column_layout = QVBoxLayout(left_column_widget)
        config_speeds_group = QGroupBox("Robot Config: Speeds & Rates")
        config_speeds_form = QFormLayout(config_speeds_group)
        self.cfg_max_linear_speed_input = QDoubleSpinBox();
        self.cfg_max_linear_speed_input.setRange(1, 50);
        self.cfg_max_linear_speed_input.setValue(GUI_REF_MAX_LINEAR_SPEED)
        config_speeds_form.addRow("Max Linear Speed (cm/s):", self.cfg_max_linear_speed_input)
        self.cfg_max_yaw_rate_input = QDoubleSpinBox();
        self.cfg_max_yaw_rate_input.setRange(0.1, math.pi);
        self.cfg_max_yaw_rate_input.setDecimals(2);
        self.cfg_max_yaw_rate_input.setValue(GUI_REF_MAX_YAW_RATE)
        config_speeds_form.addRow("Max Yaw Rate (rad/s):", self.cfg_max_yaw_rate_input)
        self.cfg_pose_linear_speed_input = QDoubleSpinBox();
        self.cfg_pose_linear_speed_input.setRange(0.1, 10);
        self.cfg_pose_linear_speed_input.setValue(GUI_REF_POSE_ADJUST_SPEED_LINEAR)
        config_speeds_form.addRow("Pose Adjust Lin Spd (cm/s):", self.cfg_pose_linear_speed_input)
        self.cfg_pose_angular_speed_input = QDoubleSpinBox();
        self.cfg_pose_angular_speed_input.setRange(1, 90);
        self.cfg_pose_angular_speed_input.setValue(math.degrees(GUI_REF_POSE_ADJUST_SPEED_ANGULAR))
        config_speeds_form.addRow("Pose Adjust Ang Spd (deg/s):", self.cfg_pose_angular_speed_input)
        send_speeds_config_button = QPushButton("Send Speed Configs");
        send_speeds_config_button.clicked.connect(self.send_speed_configs)
        config_speeds_form.addRow(send_speeds_config_button)
        left_column_layout.addWidget(config_speeds_group)

        config_gait_group = QGroupBox("Robot Config: Gait Parameters")
        config_gait_form = QFormLayout(config_gait_group)
        self.cfg_step_height_input = QDoubleSpinBox();
        self.cfg_step_height_input.setRange(0, 10);
        self.cfg_step_height_input.setValue(3.0)
        config_gait_form.addRow("Step Height (cm):", self.cfg_step_height_input)
        self.cfg_step_time_input = QDoubleSpinBox();
        self.cfg_step_time_input.setRange(0.1, 5);
        self.cfg_step_time_input.setValue(1.0)
        config_gait_form.addRow("Step Time (s):", self.cfg_step_time_input)
        send_gait_config_button = QPushButton("Send Gait Config");
        send_gait_config_button.clicked.connect(self.send_gait_configs)
        config_gait_form.addRow(send_gait_config_button)
        left_column_layout.addWidget(config_gait_group)

        extension_group = QGroupBox("Robot Config: Leg Extensions (cm)")
        extension_form_layout = QFormLayout(extension_group)
        self.extension_inputs = []
        for i in range(LEG_COUNT):
            ext_input = QDoubleSpinBox();
            ext_input.setRange(-5, 10);
            ext_input.setDecimals(1);
            ext_input.setValue(0.0)
            self.extension_inputs.append(ext_input)
            ext_input.valueChanged.connect(self.update_gui_calculated_base_pos_display)  # Update display on change
            extension_form_layout.addRow(f"Leg {i} ({LEG_NAMES[i]}):", ext_input)
        reset_ext_button = QPushButton("Reset All to 0");
        reset_ext_button.clicked.connect(self.reset_extensions_ui_and_update_display)
        send_ext_config_button = QPushButton("Send Extensions Config");
        send_ext_config_button.clicked.connect(self.send_leg_extensions_config)
        extension_form_layout.addRow(reset_ext_button, send_ext_config_button)
        left_column_layout.addWidget(extension_group)

        sys_cmd_group = QGroupBox("System Commands")
        sys_cmd_layout = QVBoxLayout(sys_cmd_group)
        self.pwm_reset_button = QPushButton("Re-initialize PWM Drivers")
        self.pwm_reset_button.clicked.connect(self.send_pwm_reset_command)
        sys_cmd_layout.addWidget(self.pwm_reset_button)
        left_column_layout.addWidget(sys_cmd_group)
        left_column_layout.addStretch()
        control_display_layout.addWidget(left_column_widget, 1)

        center_column_widget = QWidget()
        center_column_layout = QVBoxLayout(center_column_widget)
        instructions_group = QGroupBox("Keyboard Controls (Intents)")
        instructions_layout = QVBoxLayout(instructions_group)
        instructions_text = (
            "W/S: Fwd/Back\nA/D: Strafe L/R\nQ/E: Turn L/R\nSPACE: Toggle Walk Cmd\n"
            "--- Body Pose Adjust ---\nArrows: Move Body XY\nShift+Arrows: Move Body Z\n"
            "I/K: Pitch\nJ/L: Roll\nU/O: Body Yaw\n"
            "C: Center XY (Hold)\nX: Center Orientation (Hold)"
        )
        instructions_label = QLabel(instructions_text);
        instructions_label.setWordWrap(True)
        instructions_layout.addWidget(instructions_label)
        center_column_layout.addWidget(instructions_group)

        telemetry_config_group = QGroupBox("Telemetry Subscriptions")
        telemetry_config_form = QFormLayout(telemetry_config_group)
        self.sub_battery_check = QCheckBox("Battery");
        self.sub_battery_check.setChecked(True)
        self.sub_battery_interval = QLineEdit("1000");
        self.sub_battery_interval.setFixedWidth(50)
        telemetry_config_group = QGroupBox("Telemetry Subscriptions")
        telemetry_config_form = QFormLayout(telemetry_config_group)

        self.sub_battery_check = QCheckBox("Battery");
        self.sub_battery_check.setChecked(True)
        self.sub_battery_interval = QLineEdit("1000");
        self.sub_battery_interval.setFixedWidth(50)
        # Create a layout for the interval input
        battery_interval_layout = QHBoxLayout()
        battery_interval_layout.addWidget(QLabel("Interval(ms):"))
        battery_interval_layout.addWidget(self.sub_battery_interval)
        telemetry_config_form.addRow(self.sub_battery_check, battery_interval_layout)

        self.sub_robot_status_check = QCheckBox("Robot Status");
        self.sub_robot_status_check.setChecked(True)
        self.sub_robot_status_interval = QLineEdit("5000");
        self.sub_robot_status_interval.setFixedWidth(50)
        # Create a layout for the interval input
        status_interval_layout = QHBoxLayout()
        status_interval_layout.addWidget(QLabel("Interval(ms):"))
        status_interval_layout.addWidget(self.sub_robot_status_interval)
        telemetry_config_form.addRow(self.sub_robot_status_check, status_interval_layout)

        self.sub_robot_state_actual_check = QCheckBox("Robot State (Pose/Vel)");
        self.sub_robot_state_actual_check.setChecked(True)
        self.sub_robot_state_actual_interval = QLineEdit("200");
        self.sub_robot_state_actual_interval.setFixedWidth(50)
        # Create a layout for the interval input
        state_actual_interval_layout = QHBoxLayout()
        state_actual_interval_layout.addWidget(QLabel("Interval(ms):"))
        state_actual_interval_layout.addWidget(self.sub_robot_state_actual_interval)
        telemetry_config_form.addRow(self.sub_robot_state_actual_check, state_actual_interval_layout)

        self.sub_debug_foot_pos_check = QCheckBox("Debug Foot Positions");
        self.sub_debug_foot_pos_check.setChecked(False)  # Default OFF
        self.sub_debug_foot_pos_interval = QLineEdit("200");
        self.sub_debug_foot_pos_interval.setFixedWidth(50)
        debug_fp_interval_layout = QHBoxLayout()
        debug_fp_interval_layout.addWidget(QLabel("Interval(ms):"))
        debug_fp_interval_layout.addWidget(self.sub_debug_foot_pos_interval)
        telemetry_config_form.addRow(self.sub_debug_foot_pos_check, debug_fp_interval_layout)

        update_subs_button = QPushButton("Update Telemetry Subscriptions");
        update_subs_button.clicked.connect(self.send_telemetry_subscriptions)
        telemetry_config_form.addRow(update_subs_button)
        center_column_layout.addWidget(telemetry_config_group)
        center_column_layout.addStretch()
        control_display_layout.addWidget(center_column_widget, 1)

        right_column_widget = QWidget()
        right_column_layout = QVBoxLayout(right_column_widget)
        actual_state_group = QGroupBox("Robot Actual State (from Telemetry)")
        actual_state_form = QFormLayout(actual_state_group)
        actual_state_form.addRow("Cmd Walk Active:", self.actual_walk_status_label)  # Reflects commanded state
        actual_state_form.addRow("Actual Vel X:", self.actual_velocity_x_label)
        actual_state_form.addRow("Actual Vel Y:", self.actual_velocity_y_label)
        actual_state_form.addRow("Actual Yaw Rate:", self.actual_yaw_rate_label)
        actual_state_form.addRow("Actual Pos X:", self.actual_pos_x_label)
        actual_state_form.addRow("Actual Pos Y:", self.actual_pos_y_label)
        actual_state_form.addRow("Actual Pos Z:", self.actual_pos_z_label)
        actual_state_form.addRow("Actual Orient W:", self.actual_orient_w_label)
        actual_state_form.addRow("Actual Orient X:", self.actual_orient_x_label)
        actual_state_form.addRow("Actual Orient Y:", self.actual_orient_y_label)
        actual_state_form.addRow("Actual Orient Z:", self.actual_orient_z_label)
        actual_state_form.addRow("Actual Gait Height:", self.actual_gait_h_label)
        actual_state_form.addRow("Actual Gait Time:", self.actual_gait_t_label)
        right_column_layout.addWidget(actual_state_group)

        robot_telemetry_group = QGroupBox("Other Robot Telemetry")
        robot_telemetry_form = QFormLayout(robot_telemetry_group)
        robot_telemetry_form.addRow("Battery Voltage:", self.actual_battery_voltage_label)
        robot_telemetry_form.addRow("Robot Status:", self.actual_robot_status_label)
        right_column_layout.addWidget(robot_telemetry_group)

        # Create a GroupBox for Debug Foot Positions (Walk Frame)
        self.debug_foot_pos_group = QGroupBox("Debug: Foot Positions (Walk Frame - cm)")
        debug_fp_form = QFormLayout(self.debug_foot_pos_group)
        self.debug_fp_walk_labels = []  # Initialize as list
        for i in range(LEG_COUNT):
            row_layout = QHBoxLayout()
            lx = QLabel("X: N/A")
            ly = QLabel("Y: N/A")
            lz = QLabel("Z: N/A")
            row_layout.addWidget(lx);
            row_layout.addWidget(ly);
            row_layout.addWidget(lz)
            self.debug_fp_walk_labels.append({'x': lx, 'y': ly, 'z': lz})
            debug_fp_form.addRow(f"Leg {i} ({LEG_NAMES[i]}):", row_layout)

        right_column_layout.addWidget(self.debug_foot_pos_group)
        self.debug_foot_pos_group.setVisible(False)  # Initially hidden

        gui_base_pos_group = QGroupBox("GUI Calculated Base Foot Positions (to be sent)")
        gui_base_pos_form = QFormLayout(gui_base_pos_group)
        self.gui_base_pos_labels = []
        for i in range(LEG_COUNT):
            rowLayout = QHBoxLayout();
            lx = QLabel("X:0.0");
            ly = QLabel("Y:0.0");
            lz = QLabel("Z:0.0")
            rowLayout.addWidget(lx);
            rowLayout.addWidget(ly);
            rowLayout.addWidget(lz)
            self.gui_base_pos_labels.append({'x': lx, 'y': ly, 'z': lz})
            gui_base_pos_form.addRow(f"Leg {i} ({LEG_NAMES[i]}):", rowLayout)
        right_column_layout.addWidget(gui_base_pos_group)
        self.update_gui_calculated_base_pos_display()

        right_column_layout.addStretch()
        control_display_layout.addWidget(right_column_widget, 1)

        main_scroll_layout.addLayout(control_display_layout)
        main_layout.addWidget(scroll_area)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    @Slot()
    def toggle_connection(self):
        if self.hexapod_client is None:  # Try to Connect
            try:
                robot_ip = self.robot_ip_input.text()
                robot_port = int(self.robot_port_input.text())
                self.hexapod_client = HexapodUDPClient(
                    target_ip=robot_ip, target_port=robot_port,
                    gui_telemetry_listen_port=DEFAULT_GUI_TELEMETRY_LISTEN_PORT
                )
                self.send_telemetry_subscriptions()  # Send initial subscription prefs
                time.sleep(0.05)
                self.send_all_configs()  # Send initial full config state from GUI
                time.sleep(0.05)

                self.telemetry_receiver = TelemetryReceiver(DEFAULT_GUI_TELEMETRY_LISTEN_IP,
                                                            DEFAULT_GUI_TELEMETRY_LISTEN_PORT)
                self.telemetry_receiver.telemetry_received_signal.connect(self.handle_telemetry)
                self.telemetry_thread = threading.Thread(target=self.telemetry_receiver.start_listening, daemon=True)
                self.telemetry_thread.start()

                self.connection_status_label.setText(f"Status: Connected to {robot_ip}:{robot_port}")
                self.connection_status_label.setStyleSheet("color: green")
                self.connect_button.setText("Disconnect")
                self.robot_ip_input.setEnabled(False);
                self.robot_port_input.setEnabled(False)
                # self.update_freq_input.setEnabled(False) # Keep enabled if user wants to change send rate
                self.update_timer.start(int(1000.0 / self.update_freq_input.value()))
            except Exception as e:
                self.connection_status_label.setText(f"Status: Connect Error - {e}")
                self.connection_status_label.setStyleSheet("color: red")
                if self.hexapod_client: self.hexapod_client.close(); self.hexapod_client = None
                if self.telemetry_receiver: self.telemetry_receiver.stop_listening()
                if self.telemetry_thread and self.telemetry_thread.is_alive(): self.telemetry_thread.join(timeout=0.2)
        else:  # Disconnect
            self.update_timer.stop()
            if self.hexapod_client: self.hexapod_client.close()
            self.hexapod_client = None
            if self.telemetry_receiver: self.telemetry_receiver.stop_listening()
            if self.telemetry_thread and self.telemetry_thread.is_alive(): self.telemetry_thread.join(timeout=0.5)
            self.connection_status_label.setText("Status: Disconnected");
            self.connection_status_label.setStyleSheet("")
            self.connect_button.setText("Connect")
            self.robot_ip_input.setEnabled(True);
            self.robot_port_input.setEnabled(True)
            # self.update_freq_input.setEnabled(True)

    @Slot(dict)
    def handle_telemetry(self, telemetry_json):
        payload = telemetry_json.get("payload", {})
        msg_type = telemetry_json.get("type", "")

        if msg_type == "telemetry_data":
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
        elif msg_type == "robot_state_telemetry":
            if "locomotion_actual" in payload:
                loco = payload["locomotion_actual"]
                self.actual_velocity_x_label.setText(f"{loco.get('velocity_x_cms', 0):.2f} cm/s")
                self.actual_velocity_y_label.setText(f"{loco.get('velocity_y_cms', 0):.2f} cm/s")
                self.actual_yaw_rate_label.setText(f"{loco.get('angular_velocity_yaw_rads', 0):.3f} rad/s")
            if "body_pose_actual" in payload:
                pose = payload["body_pose_actual"]
                if "position_offset_cm" in pose:
                    pos = pose["position_offset_cm"]
                    self.actual_pos_x_label.setText(f"{pos.get('x', 0):.2f} cm")
                    self.actual_pos_y_label.setText(f"{pos.get('y', 0):.2f} cm")
                    self.actual_pos_z_label.setText(f"{pos.get('z', 0):.2f} cm")
                if "orientation_quat" in pose:
                    orient = pose["orientation_quat"]
                    self.actual_orient_w_label.setText(f"{orient.get('w', 1):.3f}")
                    self.actual_orient_x_label.setText(f"{orient.get('x', 0):.3f}")
                    self.actual_orient_y_label.setText(f"{orient.get('y', 0):.3f}")
                    self.actual_orient_z_label.setText(f"{orient.get('z', 0):.3f}")
            if "gait_actual" in payload:
                gait = payload["gait_actual"]
                self.actual_gait_h_label.setText(f"{gait.get('step_height_cm', 0):.1f} cm")
                self.actual_gait_t_label.setText(f"{gait.get('step_time_s', 0):.1f} s")
                is_active = gait.get('walk_active', False)
                self.actual_walk_status_label.setText("ACTIVE" if is_active else "NOT ACTIVE")
                self.actual_walk_status_label.setStyleSheet("color: green" if is_active else "color: red")

            if "debug_foot_pos_walk_cm" in payload:
                fp_walk_data = payload["debug_foot_pos_walk_cm"]
                if isinstance(fp_walk_data, list) and self.debug_foot_pos_group:  # Check if group exists
                    self.debug_foot_pos_group.setVisible(True)  # Ensure visible if data arrives
                    for i in range(min(len(fp_walk_data), LEG_COUNT)):
                        if i < len(self.debug_fp_walk_labels):
                            leg_data = fp_walk_data[i]
                            if isinstance(leg_data, dict):
                                x_val = leg_data.get('x', 'N/A')
                                y_val = leg_data.get('y', 'N/A')
                                z_val = leg_data.get('z', 'N/A')
                                self.debug_fp_walk_labels[i]['x'].setText(
                                    f"X:{x_val:.2f}" if isinstance(x_val, float) else f"X:{x_val}")
                                self.debug_fp_walk_labels[i]['y'].setText(
                                    f"Y:{y_val:.2f}" if isinstance(y_val, float) else f"Y:{y_val}")
                                self.debug_fp_walk_labels[i]['z'].setText(
                                    f"Z:{z_val:.2f}" if isinstance(z_val, float) else f"Z:{z_val}")
            elif self.debug_foot_pos_group and not self.sub_debug_foot_pos_check.isChecked():
                # If data is not present AND subscription is off, hide the group
                self.debug_foot_pos_group.setVisible(False)
                # Optionally clear labels if you want them to reset when hidden
                # for i in range(LEG_COUNT):
                #     if i < len(self.debug_fp_walk_labels):
                #         self.debug_fp_walk_labels[i]['x'].setText("X: N/A")
                #         self.debug_fp_walk_labels[i]['y'].setText("Y: N/A")
                #         self.debug_fp_walk_labels[i]['z'].setText("Z: N/A")

    @Slot()
    def update_timer_interval_ui_action(self):  # Renamed to avoid conflict
        if self.hexapod_client and self.update_timer.isActive():
            try:
                freq_hz = self.update_freq_input.value()
                if freq_hz < MIN_UPDATE_FREQUENCY_HZ: freq_hz = MIN_UPDATE_FREQUENCY_HZ
                self.update_timer.setInterval(int(1000.0 / freq_hz))
                print(f"Intent send interval: {int(1000.0 / freq_hz)} ms")
            except:
                pass

    def keyPressEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():  # Process key down only once
            self.keys_pressed.add(event.key())
            self.update_intent_factors_from_keys()
            if event.key() == Qt.Key.Key_Space and self.hexapod_client:
                self.walk_command_active = not self.walk_command_active
                self.hexapod_client.send_gait_command(self.walk_command_active)
                # Update local display immediately for responsiveness
                self.actual_walk_status_label.setText("ACTIVE" if self.walk_command_active else "NOT ACTIVE")
                self.actual_walk_status_label.setStyleSheet(
                    "color: green" if self.walk_command_active else "color: red")

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            try:
                self.keys_pressed.remove(event.key())
            except KeyError:
                pass
            self.update_intent_factors_from_keys()

    def update_intent_factors_from_keys(self):
        # This now just updates the internal intent factors. Sending happens in send_active_intents()
        self.loco_intent_vy_factor = (1.0 if Qt.Key.Key_W in self.keys_pressed else 0.0) + \
                                     (-1.0 if Qt.Key.Key_S in self.keys_pressed else 0.0)
        self.loco_intent_vx_factor = (-1.0 if Qt.Key.Key_A in self.keys_pressed else 0.0) + \
                                     (1.0 if Qt.Key.Key_D in self.keys_pressed else 0.0)
        self.loco_intent_yaw_factor = (-1.0 if Qt.Key.Key_E in self.keys_pressed else 0.0) + \
                                      (1.0 if Qt.Key.Key_Q in self.keys_pressed else 0.0)

        modifiers = QApplication.keyboardModifiers()
        shift = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)
        self.pose_adjust_intent_offset_y = (1.0 if Qt.Key.Key_Up in self.keys_pressed and not shift else 0.0) + \
                                           (-1.0 if Qt.Key.Key_Down in self.keys_pressed and not shift else 0.0)
        self.pose_adjust_intent_offset_x = (-1.0 if Qt.Key.Key_Left in self.keys_pressed else 0.0) + \
                                           (1.0 if Qt.Key.Key_Right in self.keys_pressed else 0.0)
        self.pose_adjust_intent_offset_z = (1.0 if Qt.Key.Key_Up in self.keys_pressed and shift else 0.0) + \
                                           (-1.0 if Qt.Key.Key_Down in self.keys_pressed and shift else 0.0)
        self.pose_adjust_intent_pitch = (1.0 if Qt.Key.Key_K in self.keys_pressed else 0.0) + \
                                        (-1.0 if Qt.Key.Key_I in self.keys_pressed else 0.0)
        self.pose_adjust_intent_roll = (-1.0 if Qt.Key.Key_J in self.keys_pressed else 0.0) + \
                                       (1.0 if Qt.Key.Key_L in self.keys_pressed else 0.0)
        self.pose_adjust_intent_body_yaw = (-1.0 if Qt.Key.Key_O in self.keys_pressed else 0.0) + \
                                           (1.0 if Qt.Key.Key_U in self.keys_pressed else 0.0)
        self.centering_xy_active = Qt.Key.Key_C in self.keys_pressed
        self.centering_orientation_active = Qt.Key.Key_X in self.keys_pressed

    @Slot()
    def send_active_intents(self):
        if not self.hexapod_client: return
        self.hexapod_client.send_locomotion_intent(
            self.loco_intent_vx_factor, self.loco_intent_vy_factor, self.loco_intent_yaw_factor
        )
        self.hexapod_client.send_pose_adjust_intent(
            self.pose_adjust_intent_offset_x, self.pose_adjust_intent_offset_y, self.pose_adjust_intent_offset_z,
            self.pose_adjust_intent_pitch, self.pose_adjust_intent_roll, self.pose_adjust_intent_body_yaw
        )
        if self.centering_xy_active or self.centering_orientation_active or \
                any(abs(f) > 1e-3 for f in
                    [self.loco_intent_vx_factor, self.loco_intent_vy_factor, self.loco_intent_yaw_factor,
                     self.pose_adjust_intent_offset_x, self.pose_adjust_intent_offset_y,
                     self.pose_adjust_intent_offset_z,
                     self.pose_adjust_intent_pitch, self.pose_adjust_intent_roll, self.pose_adjust_intent_body_yaw]):
            # Only send centering if it's active OR other intents are active, to avoid spamming "centering off"
            # Actually, ESP32 expects centering to be explicitly turned off.
            self.hexapod_client.send_centering_intent(self.centering_xy_active, self.centering_orientation_active)

    @Slot()
    def send_speed_configs(self):
        if not self.hexapod_client: return
        max_speeds = {"linear_cms": self.cfg_max_linear_speed_input.value(),
                      "yaw_rads": self.cfg_max_yaw_rate_input.value()}
        pose_speeds = {"linear_cms": self.cfg_pose_linear_speed_input.value(),
                       "angular_rads": math.radians(self.cfg_pose_angular_speed_input.value())}
        self.hexapod_client.send_config_update(max_speeds=max_speeds, pose_adjust_speeds=pose_speeds)
        print("Sent speed/rate configurations.")

    @Slot()
    def send_gait_configs(self):
        if not self.hexapod_client: return
        gait_p = {"step_height_cm": self.cfg_step_height_input.value(), "step_time_s": self.cfg_step_time_input.value()}
        self.hexapod_client.send_config_update(gait_params=gait_p)
        print("Sent gait configurations.")

    @Slot()
    def send_leg_extensions_config(self):
        if not self.hexapod_client: return
        base_pos = []
        for i in range(LEG_COUNT):
            ext = self.extension_inputs[i].value();
            default = DEFAULT_BASE_POSITIONS_FOR_GUI_CALC[i];
            angle = LEG_MOUNTING_ANGLES_FOR_GUI_CALC[i]
            dx, dy = math.cos(angle), math.sin(angle)
            base_pos.append(
                {"x": round(default[0] + ext * dx, 2), "y": round(default[1] + ext * dy, 2), "z": round(default[2], 2)})
        self.hexapod_client.send_config_update(base_foot_positions=base_pos)
        self.update_gui_calculated_base_pos_display(base_pos)
        print("Sent leg extensions configuration.")

    @Slot()
    def reset_extensions_ui_and_update_display(self):  # Combined
        for i in range(LEG_COUNT): self.extension_inputs[i].setValue(0.0)
        self.update_gui_calculated_base_pos_display()

    def update_gui_calculated_base_pos_display(self, positions_to_show=None):
        if positions_to_show is None:
            positions_to_show = []
            for i in range(LEG_COUNT):
                ext = self.extension_inputs[i].value();
                df = DEFAULT_BASE_POSITIONS_FOR_GUI_CALC[i];
                ag = LEG_MOUNTING_ANGLES_FOR_GUI_CALC[i]
                dx, dy = math.cos(ag), math.sin(ag);
                positions_to_show.append({"x": df[0] + ext * dx, "y": df[1] + ext * dy, "z": df[2]})
        for i in range(LEG_COUNT):
            pos = positions_to_show[i];
            lbls = self.gui_base_pos_labels[i]
            lbls['x'].setText(f"X:{pos['x']:.1f}");
            lbls['y'].setText(f"Y:{pos['y']:.1f}");
            lbls['z'].setText(f"Z:{pos['z']:.1f}")

    @Slot()
    def send_pwm_reset_command(self):
        if self.hexapod_client: self.hexapod_client.send_pwm_reinitialize_command(); print("Sent PWM re-init cmd.")

    @Slot()
    def send_telemetry_subscriptions(self):
        if not self.hexapod_client: return
        try:
            subs = {
                "battery": {"enabled": self.sub_battery_check.isChecked(),
                            "interval_ms": int(self.sub_battery_interval.text())},
                "robot_status": {"enabled": self.sub_robot_status_check.isChecked(),
                                 "interval_ms": int(self.sub_robot_status_interval.text())},
                "robot_state_actual": {"enabled": self.sub_robot_state_actual_check.isChecked(),
                                       "interval_ms": int(self.sub_robot_state_actual_interval.text())},
                "debug_foot_pos": {"enabled": self.sub_debug_foot_pos_check.isChecked(),  # Use the new checkbox
                                   "interval_ms": int(self.sub_debug_foot_pos_interval.text())}
                # Use new interval input
            }
            self.hexapod_client.send_client_settings(subscriptions_override=subs)
            print("Sent updated telemetry subscriptions.")
            # Toggle visibility of the debug group based on subscription
            if self.debug_foot_pos_group:  # Check if it's initialized
                self.debug_foot_pos_group.setVisible(self.sub_debug_foot_pos_check.isChecked())
        except ValueError:
            print("[ERROR] Invalid interval value for telemetry subscriptions. Must be integer.")
            if self.debug_foot_pos_group:
                self.debug_foot_pos_group.setVisible(False)  # Hide on error too

    def send_all_configs(self):
        self.send_speed_configs();
        time.sleep(0.02)
        self.send_gait_configs();
        time.sleep(0.02)
        self.send_leg_extensions_config()

    def closeEvent(self, event: QCloseEvent):
        if self.hexapod_client:
            self.toggle_connection()  # Handles client.close() and thread stop
        elif self.telemetry_receiver:
            self.telemetry_receiver.stop_listening()
        if self.telemetry_thread and self.telemetry_thread.is_alive(): self.telemetry_thread.join(timeout=0.2)
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HexapodControllerGUI()
    window.show()
    sys.exit(app.exec())