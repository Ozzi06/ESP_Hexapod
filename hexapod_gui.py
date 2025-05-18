# !/usr/bin/env python3
import sys
import math
import time
import json
import socket
import threading

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QGroupBox, QFormLayout,
    QSizePolicy, QCheckBox, QTextEdit, QScrollArea
)
from PySide6.QtCore import Qt, QTimer, Slot, QElapsedTimer, Signal, QObject
from PySide6.QtGui import QKeyEvent, QCloseEvent, QDoubleValidator, QTextCursor

try:
    from hexapod_comms_client import HexapodCommsClient

    LEG_COUNT = 6
except ImportError:
    print("CRITICAL ERROR: Could not find hexapod_comms_client.py. Ensure it is in the same directory or Python path.")
    sys.exit("Error: Could not find hexapod_comms_client.py.")

# --- Configuration ---
DEFAULT_ROBOT_TARGET_IP = "192.168.68.121"  # CHANGE THIS TO YOUR ESP32's ACTUAL IP
DEFAULT_ROBOT_TCP_PORT = 5006
DEFAULT_ROBOT_UDP_PORT = 5005
DEFAULT_GUI_TELEMETRY_LISTEN_IP = "0.0.0.0"
DEFAULT_GUI_TELEMETRY_LISTEN_PORT = 5007
DEFAULT_UPDATE_FREQUENCY_HZ = 20.0
MIN_UPDATE_FREQUENCY_HZ = 5.0
MAX_UPDATE_FREQUENCY_HZ = 50.0

# Default GUI reference values for configuration fields
# These should match the ESP32's initial static defaults in remote_control.cpp
GUI_REF_MAX_LINEAR_SPEED = 8.0
GUI_REF_MAX_YAW_RATE = 0.3
GUI_REF_POSE_ADJUST_SPEED_LINEAR = 2.0
GUI_REF_POSE_ADJUST_SPEED_ANGULAR = math.radians(15.0)  # approx 0.26 rad/s

GUI_REF_LINEAR_ACCEL_CMS2 = 10.0
GUI_REF_LINEAR_DECEL_CMS2 = 20.0
GUI_REF_YAW_ACCEL_DEGS2 = 30.0  # GUI uses deg/s^2
GUI_REF_YAW_DECEL_DEGS2 = 60.0  # GUI uses deg/s^2

GUI_REF_STEP_HEIGHT_CM = 3.0
GUI_REF_STEP_TIME_S = 1.0

LEG_NAMES = ["BR", "MR", "FR", "BL", "ML", "FL"]
# Defaults for abstract leg geometry inputs
DEFAULT_LEG_POS_FR_X_CM = 27.0
DEFAULT_LEG_POS_FR_Y_CM = 19.0
DEFAULT_LEG_POS_MR_X_CM = 32.0
DEFAULT_LEG_CORNER_EXT_CM = 0.0
DEFAULT_LEG_MIDDLE_EXT_CM = 0.0

# Leg mounting angles for GUI's calculation display (should match ESP32's legMountingAngle)
# Order: BR, MR, FR, BL, ML, FL
ESP_LEG_MOUNTING_ANGLES = [
    -1 * math.pi / 4.0,  # Leg 0 (BR)
    0 * math.pi / 4.0,  # Leg 1 (MR)
    +1 * math.pi / 4.0,  # Leg 2 (FR)
    -3 * math.pi / 4.0,  # Leg 3 (BL)
    +4 * math.pi / 4.0,  # Leg 4 (ML)
    +3 * math.pi / 4.0  # Leg 5 (FL)
]


class TelemetryReceiverUDP(QObject):
    telemetry_received_signal = Signal(dict)

    def __init__(self, listen_ip, listen_port):
        super().__init__()
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.running = False
        self.sock = None
        self.thread = None
        # print(f"TelemetryReceiverUDP initialized to listen on {listen_ip}:{listen_port}")

    def start_listening(self):
        if self.running:
            # print("UDP Telemetry listener already running.")
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
                if self.running:  # Only print if we are supposed to be running
                    print(f"[ERROR UDP RX] Socket error: {e}")
                break  # Exit loop on other socket errors
        if self.sock:
            self.sock.close()
            self.sock = None
        print("UDP Telemetry listener stopped.")

    def stop_listening(self):
        # print("UDP Telemetry stop_listening called.")
        self.running = False
        if self.thread and self.thread.is_alive():
            # print("UDP Telemetry joining thread...")
            self.thread.join(timeout=1.5)
            if self.thread.is_alive():
                print("[WARN] UDP Telemetry listener thread did not join.")
        self.thread = None
        # print("UDP Telemetry listener thread joined or was None.")


class HexapodControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hexapod Controller GUI v2 (TCP/UDP)")
        self.setGeometry(30, 30, 1400, 950)  # Adjusted for more content

        self.comms_client = None
        self.udp_intent_timer = QTimer(self)
        self.udp_intent_timer.timeout.connect(self.send_active_intents_udp)

        self.udp_telemetry_receiver = TelemetryReceiverUDP(
            DEFAULT_GUI_TELEMETRY_LISTEN_IP, DEFAULT_GUI_TELEMETRY_LISTEN_PORT
        )
        self.udp_telemetry_receiver.telemetry_received_signal.connect(self.handle_udp_telemetry)

        # --- Actual State Labels ---
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
        self.actual_walk_status_label = QLabel("N/A")  # For walkCycleRunning

        self.debug_foot_pos_group = None
        self.debug_fp_walk_labels = [{'x': QLabel("X: N/A"), 'y': QLabel("Y: N/A"), 'z': QLabel("Z: N/A")} for _ in
                                     range(LEG_COUNT)]

        # Store all config spinboxes for easy enable/disable/NA management
        self.all_config_spinboxes = []

        # --- Keyboard intent state ---
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

        # --- Top Row: Connection ---
        connection_group = QGroupBox("Connection")
        connection_layout = QHBoxLayout(connection_group)
        connection_layout.addWidget(QLabel("Robot IP:"))
        self.robot_ip_input = QLineEdit(DEFAULT_ROBOT_TARGET_IP)
        connection_layout.addWidget(self.robot_ip_input)
        connection_layout.addWidget(QLabel("TCP Port:"))
        self.robot_tcp_port_input = QLineEdit(str(DEFAULT_ROBOT_TCP_PORT));
        self.robot_tcp_port_input.setFixedWidth(60)
        self.robot_tcp_port_input.setValidator(QDoubleValidator(1024, 65535, 0))
        connection_layout.addWidget(self.robot_tcp_port_input)
        connection_layout.addWidget(QLabel("UDP Port:"))
        self.robot_udp_port_input = QLineEdit(str(DEFAULT_ROBOT_UDP_PORT));
        self.robot_udp_port_input.setFixedWidth(60)
        self.robot_udp_port_input.setValidator(QDoubleValidator(1024, 65535, 0))
        connection_layout.addWidget(self.robot_udp_port_input)
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
        connection_layout.addStretch(1)
        main_layout.addWidget(connection_group)

        # --- Main Content Area ---
        main_content_widget = QWidget()
        content_layout = QHBoxLayout(main_content_widget)

        # == Column 1: System & Controls ==
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
        instructions_label = QLabel(instr_text);
        instructions_label.setWordWrap(True)
        instr_layout_v = QVBoxLayout(instructions_group)
        instr_layout_v.addWidget(instructions_label)
        col1_layout.addWidget(instructions_group)

        col1_layout.addStretch(1)
        content_layout.addWidget(col1_widget, 2)

        # == Column 2: Robot Configuration (Speeds, Accel, Gait) ==
        col2_widget = QWidget();
        col2_layout = QVBoxLayout(col2_widget)

        config_speeds_group = QGroupBox("Config: Speeds & Rates")
        config_speeds_form = QFormLayout(config_speeds_group)
        self.cfg_max_linear_speed_input = QDoubleSpinBox();
        self.cfg_max_linear_speed_input.setRange(1, 50);
        config_speeds_form.addRow("Max Linear Speed (cm/s):", self.cfg_max_linear_speed_input)
        self.cfg_max_yaw_rate_input = QDoubleSpinBox();
        self.cfg_max_yaw_rate_input.setRange(0.01, math.pi);
        self.cfg_max_yaw_rate_input.setDecimals(2);
        config_speeds_form.addRow("Max Yaw Rate (rad/s):", self.cfg_max_yaw_rate_input)
        self.cfg_pose_linear_speed_input = QDoubleSpinBox();
        self.cfg_pose_linear_speed_input.setRange(0.1, 10);
        config_speeds_form.addRow("Pose Adjust Lin Spd (cm/s):", self.cfg_pose_linear_speed_input)
        self.cfg_pose_angular_speed_input = QDoubleSpinBox();
        self.cfg_pose_angular_speed_input.setRange(1, 90);
        config_speeds_form.addRow("Pose Adjust Ang Spd (deg/s):", self.cfg_pose_angular_speed_input)
        col2_layout.addWidget(config_speeds_group)
        self.all_config_spinboxes.extend(
            [self.cfg_max_linear_speed_input, self.cfg_max_yaw_rate_input, self.cfg_pose_linear_speed_input,
             self.cfg_pose_angular_speed_input])
        self.cfg_max_linear_speed_input.valueChanged.connect(self.send_movement_configs_auto)
        self.cfg_max_yaw_rate_input.valueChanged.connect(self.send_movement_configs_auto)
        self.cfg_pose_linear_speed_input.valueChanged.connect(self.send_movement_configs_auto)
        self.cfg_pose_angular_speed_input.valueChanged.connect(self.send_movement_configs_auto)

        config_accel_group = QGroupBox("Config: Acceleration")
        config_accel_form = QFormLayout(config_accel_group)
        self.cfg_linear_accel_input = QDoubleSpinBox();
        self.cfg_linear_accel_input.setRange(1.0, 500.0);
        self.cfg_linear_accel_input.setDecimals(1);
        config_accel_form.addRow("Linear Accel (cm/s²):", self.cfg_linear_accel_input)
        self.cfg_linear_decel_input = QDoubleSpinBox();
        self.cfg_linear_decel_input.setRange(1.0, 1000.0);
        self.cfg_linear_decel_input.setDecimals(1);
        config_accel_form.addRow("Linear Decel (cm/s²):", self.cfg_linear_decel_input)
        self.cfg_yaw_accel_input = QDoubleSpinBox();
        self.cfg_yaw_accel_input.setRange(0.1, 360.0);
        self.cfg_yaw_accel_input.setDecimals(1);
        config_accel_form.addRow("Yaw Accel (deg/s²):", self.cfg_yaw_accel_input)
        self.cfg_yaw_decel_input = QDoubleSpinBox();
        self.cfg_yaw_decel_input.setRange(0.1, 720.0);
        self.cfg_yaw_decel_input.setDecimals(1);
        config_accel_form.addRow("Yaw Decel (deg/s²):", self.cfg_yaw_decel_input)
        col2_layout.addWidget(config_accel_group)
        self.all_config_spinboxes.extend(
            [self.cfg_linear_accel_input, self.cfg_linear_decel_input, self.cfg_yaw_accel_input,
             self.cfg_yaw_decel_input])
        self.cfg_linear_accel_input.valueChanged.connect(self.send_accel_configs_auto)
        self.cfg_linear_decel_input.valueChanged.connect(self.send_accel_configs_auto)
        self.cfg_yaw_accel_input.valueChanged.connect(self.send_accel_configs_auto)
        self.cfg_yaw_decel_input.valueChanged.connect(self.send_accel_configs_auto)

        config_gait_group = QGroupBox("Config: Gait Parameters")
        config_gait_form = QFormLayout(config_gait_group)
        self.cfg_step_height_input = QDoubleSpinBox();
        self.cfg_step_height_input.setRange(0, 10);
        config_gait_form.addRow("Step Height (cm):", self.cfg_step_height_input)
        self.cfg_step_time_input = QDoubleSpinBox();
        self.cfg_step_time_input.setRange(0.1, 5);
        config_gait_form.addRow("Step Time (s):", self.cfg_step_time_input)
        col2_layout.addWidget(config_gait_group)
        self.all_config_spinboxes.extend([self.cfg_step_height_input, self.cfg_step_time_input])
        self.cfg_step_height_input.valueChanged.connect(self.send_gait_configs_auto)
        self.cfg_step_time_input.valueChanged.connect(self.send_gait_configs_auto)

        col2_layout.addStretch(1)
        content_layout.addWidget(col2_widget, 3)

        # == Column 3: Robot Config - Geometry ==
        col3_widget = QWidget();
        col3_layout = QVBoxLayout(col3_widget)
        leg_base_geom_group = QGroupBox("Config: Leg Base Geometry (Abstract, cm)")
        leg_base_geom_form = QFormLayout(leg_base_geom_group)
        self.cfg_leg_front_corner_x = QDoubleSpinBox();
        self.cfg_leg_front_corner_x.setRange(-50, 50);
        self.cfg_leg_front_corner_x.setDecimals(1)
        leg_base_geom_form.addRow("Front Corner X:", self.cfg_leg_front_corner_x)  # Units in group title
        self.cfg_leg_front_corner_y = QDoubleSpinBox();
        self.cfg_leg_front_corner_y.setRange(-50, 50);
        self.cfg_leg_front_corner_y.setDecimals(1)
        leg_base_geom_form.addRow("Front Corner Y:", self.cfg_leg_front_corner_y)  # Units in group title
        self.cfg_leg_middle_side_x = QDoubleSpinBox();
        self.cfg_leg_middle_side_x.setRange(-50, 50);
        self.cfg_leg_middle_side_x.setDecimals(1)
        leg_base_geom_form.addRow("Middle Side X:", self.cfg_leg_middle_side_x)  # Units in group title
        self.cfg_leg_corner_ext = QDoubleSpinBox();
        self.cfg_leg_corner_ext.setRange(-10, 10)
        self.cfg_leg_corner_ext.setDecimals(1)
        leg_base_geom_form.addRow("Corner Legs Ext:", self.cfg_leg_corner_ext)  # Units in group title
        self.cfg_leg_middle_ext = QDoubleSpinBox();
        self.cfg_leg_middle_ext.setRange(-10, 10);
        self.cfg_leg_middle_ext.setDecimals(1)
        leg_base_geom_form.addRow("Middle Legs Ext:", self.cfg_leg_middle_ext)  # Units in group title
        col3_layout.addWidget(leg_base_geom_group)
        self.all_config_spinboxes.extend(
            [self.cfg_leg_front_corner_x, self.cfg_leg_front_corner_y, self.cfg_leg_middle_side_x,
             self.cfg_leg_corner_ext, self.cfg_leg_middle_ext])
        self.cfg_leg_front_corner_x.valueChanged.connect(self.send_leg_geometry_configs_auto)
        self.cfg_leg_front_corner_y.valueChanged.connect(self.send_leg_geometry_configs_auto)
        self.cfg_leg_middle_side_x.valueChanged.connect(self.send_leg_geometry_configs_auto)
        self.cfg_leg_corner_ext.valueChanged.connect(self.send_leg_geometry_configs_auto)
        self.cfg_leg_middle_ext.valueChanged.connect(self.send_leg_geometry_configs_auto)

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
        content_layout.addWidget(col3_widget, 3)

        # == Column 4: Telemetry Subscriptions & Display ==
        col4_widget = QWidget();
        col4_layout = QVBoxLayout(col4_widget)
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
        col4_layout.addWidget(telemetry_config_group)

        robot_telemetry_group = QGroupBox("Other Robot Telemetry (from TCP)")
        robot_telemetry_form = QFormLayout(robot_telemetry_group)
        robot_telemetry_form.addRow("Battery Voltage:", self.actual_battery_voltage_label)
        robot_telemetry_form.addRow("Robot Status:", self.actual_robot_status_label)
        col4_layout.addWidget(robot_telemetry_group)
        col4_layout.addStretch(1)
        content_layout.addWidget(col4_widget, 3)

        # == Column 5: Actual State & Debug Display ==
        col5_widget = QWidget();
        col5_layout = QVBoxLayout(col5_widget)
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
            row_layout = QHBoxLayout()
            row_layout.addWidget(self.debug_fp_walk_labels[i]['x'])
            row_layout.addWidget(self.debug_fp_walk_labels[i]['y'])
            row_layout.addWidget(self.debug_fp_walk_labels[i]['z'])
            debug_fp_form.addRow(f"Leg {i} ({LEG_NAMES[i]}):", row_layout)
        self.debug_foot_pos_group.setVisible(False)
        self.sub_debug_foot_pos_check.stateChanged.connect(
            lambda state: self.debug_foot_pos_group.setVisible(state == Qt.CheckState.Checked.value)
        )
        col5_layout.addWidget(self.debug_foot_pos_group)

        col5_layout.addStretch(1)
        content_layout.addWidget(col5_widget, 3)

        main_layout.addWidget(main_content_widget)

        # --- Bottom Row: Terminal Log ---
        self.terminal_log_area = QTextEdit()
        self.terminal_log_area.setReadOnly(True)
        self.terminal_log_area.setFixedHeight(100)
        self.terminal_log_area.setStyleSheet(
            "background-color: #f0f0f0; color: #333; font-family: Consolas, monospace;")
        main_layout.addWidget(self.terminal_log_area)

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def create_interval_layout(self, line_edit_widget):
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Interval(ms):"))
        layout.addWidget(line_edit_widget)
        return layout

    def log_to_terminal(self, message: str):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.terminal_log_area.append(f"[{timestamp}] {message}")
        self.terminal_log_area.moveCursor(QTextCursor.MoveOperation.End)

    def set_ui_for_disconnected_state(self):
        self.log_to_terminal("UI set to disconnected state.")
        self.robot_ip_input.setEnabled(True)
        self.robot_tcp_port_input.setEnabled(True)
        self.robot_udp_port_input.setEnabled(True)
        self.connect_button.setText("Connect")
        self.fetch_state_button.setEnabled(False)
        self.connection_status_label.setText("Status: Disconnected")
        self.connection_status_label.setStyleSheet("color: black")

        for sb in self.all_config_spinboxes:
            sb.blockSignals(True)  # Block signals before changing value
            sb.setEnabled(False)
            sb.setSpecialValueText("N/A")
            # Set to a consistent value for N/A display (e.g., 0 if in range, else min)
            if sb.minimum() <= 0.0 and sb.maximum() >= 0.0:
                sb.setValue(0.0)
            else:
                sb.setValue(sb.minimum())
            sb.blockSignals(False)  # Unblock signals

        self.pwm_reset_button.setEnabled(False)
        for label in [self.actual_battery_voltage_label, self.actual_robot_status_label,
                      self.actual_velocity_x_label, self.actual_velocity_y_label, self.actual_yaw_rate_label,
                      self.actual_pos_x_label, self.actual_pos_y_label, self.actual_pos_z_label,
                      self.actual_orient_w_label, self.actual_orient_x_label, self.actual_orient_y_label,
                      self.actual_orient_z_label,
                      self.actual_gait_h_label, self.actual_gait_t_label, self.actual_walk_status_label]:
            label.setText("N/A")
        for i in range(LEG_COUNT):
            self.debug_fp_walk_labels[i]['x'].setText("X: N/A");
            self.debug_fp_walk_labels[i]['y'].setText("Y: N/A");
            self.debug_fp_walk_labels[i]['z'].setText("Z: N/A")
        self.actual_walk_status_label.setStyleSheet("color: gray")

    def set_ui_for_connected_but_unsynced_state(self):
        self.log_to_terminal("UI set to connected but unsynced state.")
        self.robot_ip_input.setEnabled(False);
        self.robot_tcp_port_input.setEnabled(False);
        self.robot_udp_port_input.setEnabled(False)
        self.connect_button.setText("Disconnect")
        self.fetch_state_button.setEnabled(True)

        for sb in self.all_config_spinboxes:
            sb.blockSignals(True)  # Block signals before changing value
            sb.setEnabled(False)
            sb.setSpecialValueText("N/A")
            # Set to a consistent value for N/A display
            if sb.minimum() <= 0.0 and sb.maximum() >= 0.0:
                sb.setValue(0.0)
            else:
                sb.setValue(sb.minimum())
            sb.blockSignals(False)  # Unblock signals

        self.pwm_reset_button.setEnabled(True)

    def set_ui_for_synced_state(self):
        self.log_to_terminal("UI set to synced state.")
        self.fetch_state_button.setEnabled(True)
        # Values are already set by populate_gui_from_full_state
        for sb in self.all_config_spinboxes:
            sb.setEnabled(True)
            sb.setSpecialValueText("")  # Clear "N/A" text
        self.pwm_reset_button.setEnabled(True)

    @Slot()
    def toggle_connection(self):
        if not self.comms_client or not self.comms_client.is_tcp_connected():
            robot_ip = self.robot_ip_input.text()
            try:
                robot_tcp_port = int(self.robot_tcp_port_input.text())
                robot_udp_port = int(self.robot_udp_port_input.text())
                if not (1024 <= robot_tcp_port <= 65535 and 1024 <= robot_udp_port <= 65535):
                    raise ValueError("Port numbers out of range.")
            except ValueError as e:
                self.log_to_terminal(f"Invalid port number(s): {e}");
                self.connection_status_label.setText("Status: Invalid Port(s)");
                self.connection_status_label.setStyleSheet("color: red");
                return

            self.connect_button.setEnabled(False);
            self.connection_status_label.setText(f"Status: Connecting to {robot_ip}...");
            self.connection_status_label.setStyleSheet("color: orange")
            self.log_to_terminal(f"Attempting connection to {robot_ip} (TCP:{robot_tcp_port}, UDP:{robot_udp_port})")

            self.comms_client = HexapodCommsClient(robot_ip, robot_tcp_port, robot_udp_port,
                                                   DEFAULT_GUI_TELEMETRY_LISTEN_IP, DEFAULT_GUI_TELEMETRY_LISTEN_PORT)
            self.comms_client.tcp_connected_signal.connect(self.handle_tcp_conn_success)
            self.comms_client.tcp_disconnected_signal.connect(self.handle_tcp_conn_loss)
            self.comms_client.tcp_message_received_signal.connect(self.handle_tcp_message_from_esp)

            if not self.comms_client.connect_tcp():
                # connect_tcp already emits tcp_disconnected_signal which calls handle_tcp_conn_loss
                # handle_tcp_conn_loss will re-enable the button and set UI.
                # So, no need to explicitly enable button here if connect_tcp fails.
                pass
        else:
            self.log_to_terminal("Disconnecting...")
            if self.comms_client:
                self.comms_client.send_disconnect_notice();
                time.sleep(0.05)  # Give a moment for the notice to be sent
                self.comms_client.close();  # This will trigger disconnect_tcp in comms_client
                self.comms_client = None
            self.udp_intent_timer.stop()
            self.udp_telemetry_receiver.stop_listening()
            self.set_ui_for_disconnected_state()  # This resets button text and status
            self.connect_button.setEnabled(True)  # Ensure button is enabled after disconnect
            self.log_to_terminal("Disconnected.")

    @Slot()
    def handle_tcp_conn_success(self):
        self.log_to_terminal(f"TCP Connection successful to {self.comms_client.robot_tcp_addr[0]}.")
        self.connection_status_label.setText("Status: TCP Connected. Initializing...");
        self.connection_status_label.setStyleSheet("color: darkGreen")

        self.set_ui_for_connected_but_unsynced_state()  # This now correctly blocks signals
        self.connect_button.setEnabled(True)  # Connection attempt finished, re-enable button (now as "Disconnect")

        self.udp_telemetry_receiver.start_listening()
        self.send_client_settings_to_robot()  # Configure telemetry on robot
        time.sleep(0.05)  # Brief pause
        self.request_full_state_from_robot()  # Request initial state
        self.update_udp_intent_timer_interval()  # Setup intent timer
        self.udp_intent_timer.start()

    @Slot(str)
    def handle_tcp_conn_loss(self, reason: str):
        self.log_to_terminal(f"TCP Connection Lost/Failed: {reason}")
        self.connection_status_label.setText(f"Status: TCP Error - {reason}");
        self.connection_status_label.setStyleSheet("color: red")

        self.udp_intent_timer.stop()
        if self.comms_client:
            # Comms client's disconnect_tcp might have already been called if error originated there.
            # Calling close() ensures all resources are released if not already.
            self.comms_client.close()
            self.comms_client = None

        self.udp_telemetry_receiver.stop_listening()
        self.set_ui_for_disconnected_state()  # Resets UI elements including button text
        self.connect_button.setEnabled(True)  # Ensure button is usable for new connection attempt

    @Slot(dict)
    def handle_tcp_message_from_esp(self, message_json: dict):
        self.log_to_terminal(f"RX TCP: {json.dumps(message_json)}")
        msg_type = message_json.get("type")
        payload = message_json.get("payload")

        if msg_type == "full_state_response" and payload:
            self.populate_gui_from_full_state(payload)
            self.connection_status_label.setText("Status: Connected & Synced");
            self.connection_status_label.setStyleSheet("color: green")
            self.set_ui_for_synced_state()  # This enables spinboxes and clears "N/A"
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

    def populate_gui_from_full_state(self, payload: dict):
        # Block signals on all config spinboxes to prevent them from firing
        # their valueChanged signals when we programmatically set their values.
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
            self.log_to_terminal("Populated GUI abstract leg geometry inputs from robot state.")

        # Base foot positions are calculated by GUI based on abstract geometry,
        # or could be received if ESP sends them directly (not currently implemented for this field from ESP to GUI state)
        # if "base_foot_positions_walk_cm" in payload and isinstance(payload["base_foot_positions_walk_cm"], list):
        #     pass # Example: Could populate some other display if needed

        if "current_body_pose" in payload:  # This is more for actual telemetry, but could be part of full state
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

        walk_is_active = payload.get("walk_active", False)  # From full state
        self.update_walk_active_display(walk_is_active)

        # Unblock signals now that values are set
        for sb in self.all_config_spinboxes: sb.blockSignals(False)

        # Update GUI calculated display based on newly populated abstract geometry
        self.update_gui_calculated_base_pos_display()

    def update_walk_active_display(self, is_active: bool):
        self.actual_walk_status_label.setText("ACTIVE" if is_active else "NOT ACTIVE")
        self.actual_walk_status_label.setStyleSheet("color: green" if is_active else "color: red")

    @Slot(dict)
    def handle_udp_telemetry(self, telemetry_json: dict):
        payload = telemetry_json.get("payload", {})
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
                    self.actual_pos_x_label.setText(f"{pos.get('x', 0):.2f} cm")
                    self.actual_pos_y_label.setText(f"{pos.get('y', 0):.2f} cm")
                    self.actual_pos_z_label.setText(f"{pos.get('z', 0):.2f} cm")
                if "orientation_quat" in pose:
                    orient = pose["orientation_quat"]
                    self.actual_orient_w_label.setText(f"{orient.get('w', 1):.3f}")
                    self.actual_orient_x_label.setText(f"{orient.get('x', 0):.3f}")
                    self.actual_orient_y_label.setText(f"{orient.get('y', 0):.3f}")
                    self.actual_orient_z_label.setText(f"{orient.get('z', 0):.3f}")
            if "gait_actual" in payload:  # Gait params can also come from UDP telemetry
                gait = payload["gait_actual"]
                # Update actual display labels
                self.actual_gait_h_label.setText(f"{gait.get('step_height_cm', 0):.1f} cm")
                self.actual_gait_t_label.setText(f"{gait.get('step_time_s', 0):.1f} s")
                self.update_walk_active_display(gait.get('walk_active', False))  # walk_active status from telemetry

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
                if freq_hz < MIN_UPDATE_FREQUENCY_HZ: freq_hz = MIN_UPDATE_FREQUENCY_HZ
                if freq_hz > MAX_UPDATE_FREQUENCY_HZ: freq_hz = MAX_UPDATE_FREQUENCY_HZ  # Ensure max is also respected
                self.udp_intent_timer.setInterval(int(1000.0 / freq_hz))
            except ZeroDivisionError:
                self.udp_intent_timer.setInterval(int(1000.0 / DEFAULT_UPDATE_FREQUENCY_HZ))
            except Exception:
                pass  # Or log error

    def keyPressEvent(self, event: QKeyEvent):
        if not self.comms_client or not self.comms_client.is_tcp_connected(): return
        if not event.isAutoRepeat():
            self.keys_pressed.add(event.key());
            self.update_intent_factors_from_keys()
            if event.key() == Qt.Key.Key_Space:
                current_walk_state_str = self.actual_walk_status_label.text()
                robot_is_walking = (current_walk_state_str == "ACTIVE")
                new_desired_state = not robot_is_walking
                if self.comms_client.send_gait_command(new_desired_state):
                    self.log_to_terminal(f"TX TCP: GaitCommand walk_active: {new_desired_state}")
                    # Optimistically update GUI, or wait for telemetry confirmation
                    # self.update_walk_active_display(new_desired_state) # Optional: optimistic update

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            try:
                self.keys_pressed.remove(event.key())
            except KeyError:
                pass
            self.update_intent_factors_from_keys()

    def update_intent_factors_from_keys(self):
        self.loco_intent_vy_factor = (1.0 if Qt.Key.Key_W in self.keys_pressed else 0.0) + \
                                     (-1.0 if Qt.Key.Key_S in self.keys_pressed else 0.0)
        self.loco_intent_vx_factor = (-1.0 if Qt.Key.Key_A in self.keys_pressed else 0.0) + \
                                     (1.0 if Qt.Key.Key_D in self.keys_pressed else 0.0)
        self.loco_intent_yaw_factor = (1.0 if Qt.Key.Key_E in self.keys_pressed else 0.0) + \
                                      (-1.0 if Qt.Key.Key_Q in self.keys_pressed else 0.0)

        modifiers = QApplication.keyboardModifiers();
        shift_pressed = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)

        # Pose adjust X/Y (arrows)
        self.pose_adjust_intent_offset_y = 0.0
        if Qt.Key.Key_Up in self.keys_pressed and not shift_pressed: self.pose_adjust_intent_offset_y = 1.0
        if Qt.Key.Key_Down in self.keys_pressed and not shift_pressed: self.pose_adjust_intent_offset_y = -1.0

        self.pose_adjust_intent_offset_x = 0.0
        if Qt.Key.Key_Left in self.keys_pressed and not shift_pressed: self.pose_adjust_intent_offset_x = -1.0
        if Qt.Key.Key_Right in self.keys_pressed and not shift_pressed: self.pose_adjust_intent_offset_x = 1.0

        # Pose adjust Z (Shift + arrows Up/Down)
        self.pose_adjust_intent_offset_z = 0.0
        if Qt.Key.Key_Up in self.keys_pressed and shift_pressed: self.pose_adjust_intent_offset_z = 1.0
        if Qt.Key.Key_Down in self.keys_pressed and shift_pressed: self.pose_adjust_intent_offset_z = -1.0

        self.pose_adjust_intent_pitch = (1.0 if Qt.Key.Key_K in self.keys_pressed else 0.0) + \
                                        (-1.0 if Qt.Key.Key_I in self.keys_pressed else 0.0)
        self.pose_adjust_intent_roll = (-1.0 if Qt.Key.Key_J in self.keys_pressed else 0.0) + \
                                       (1.0 if Qt.Key.Key_L in self.keys_pressed else 0.0)
        self.pose_adjust_intent_body_yaw = (-1.0 if Qt.Key.Key_O in self.keys_pressed else 0.0) + \
                                           (1.0 if Qt.Key.Key_U in self.keys_pressed else 0.0)

        self.centering_xy_active = Qt.Key.Key_C in self.keys_pressed
        self.centering_orientation_active = Qt.Key.Key_X in self.keys_pressed

    @Slot()
    def send_active_intents_udp(self):
        if not self.comms_client or not self.comms_client.is_tcp_connected(): return  # Guard: only send if connected

        self.comms_client.send_locomotion_intent(self.loco_intent_vx_factor, self.loco_intent_vy_factor,
                                                 self.loco_intent_yaw_factor)
        self.comms_client.send_pose_adjust_intent(self.pose_adjust_intent_offset_x, self.pose_adjust_intent_offset_y,
                                                  self.pose_adjust_intent_offset_z, self.pose_adjust_intent_pitch,
                                                  self.pose_adjust_intent_roll, self.pose_adjust_intent_body_yaw)
        self.comms_client.send_centering_intent(self.centering_xy_active, self.centering_orientation_active)

    @Slot()
    def send_movement_configs_auto(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            # Check if the sender is enabled (i.e., GUI is in synced state)
            sender_spinbox = self.sender()
            if sender_spinbox and not sender_spinbox.isEnabled():
                # self.log_to_terminal("DEBUG: Movement config change ignored (sender disabled).")
                return

            max_speeds = {"linear_cms": self.cfg_max_linear_speed_input.value(),
                          "yaw_rads": self.cfg_max_yaw_rate_input.value()}
            pose_speeds = {"linear_cms": self.cfg_pose_linear_speed_input.value(),
                           "angular_rads": math.radians(self.cfg_pose_angular_speed_input.value())}
            if self.comms_client.send_config_update(max_speeds=max_speeds, pose_adjust_speeds=pose_speeds):
                self.log_to_terminal(f"TX TCP Config: Speeds & Pose Rates Update")

    @Slot()
    def send_accel_configs_auto(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            sender_spinbox = self.sender()
            if sender_spinbox and not sender_spinbox.isEnabled():
                # self.log_to_terminal("DEBUG: Accel config change ignored (sender disabled).")
                return

            accel_payload = {
                "linear_accel_cms2": self.cfg_linear_accel_input.value(),
                "linear_decel_cms2": self.cfg_linear_decel_input.value(),
                "yaw_accel_rads2": math.radians(self.cfg_yaw_accel_input.value()),
                "yaw_decel_rads2": math.radians(self.cfg_yaw_decel_input.value())
            }
            if self.comms_client.send_config_update(acceleration_values=accel_payload):
                self.log_to_terminal(f"TX TCP Config: Acceleration Values Update")

    @Slot()
    def send_gait_configs_auto(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            sender_spinbox = self.sender()
            if sender_spinbox and not sender_spinbox.isEnabled():
                # self.log_to_terminal("DEBUG: Gait config change ignored (sender disabled).")
                return

            gait_p = {"step_height_cm": self.cfg_step_height_input.value(),
                      "step_time_s": self.cfg_step_time_input.value()}
            if self.comms_client.send_config_update(gait_params=gait_p):
                self.log_to_terminal(f"TX TCP Config: Gait Update")

    @Slot()
    def send_leg_geometry_configs_auto(self):
        self.update_gui_calculated_base_pos_display()  # Update GUI display immediately
        if self.comms_client and self.comms_client.is_tcp_connected():
            sender_spinbox = self.sender()
            if sender_spinbox and not sender_spinbox.isEnabled():
                # self.log_to_terminal("DEBUG: Leg Geom config change ignored (sender disabled).")
                return

            abstract_geom_payload = {
                "front_corner_x_cm": self.cfg_leg_front_corner_x.value(),
                "front_corner_y_cm": self.cfg_leg_front_corner_y.value(),
                "middle_side_x_cm": self.cfg_leg_middle_side_x.value(),
                "corner_ext_cm": self.cfg_leg_corner_ext.value(),
                "middle_ext_cm": self.cfg_leg_middle_ext.value()
            }
            if self.comms_client.send_config_update(leg_geometry_abstract=abstract_geom_payload):
                self.log_to_terminal(f"TX TCP Config: Abstract Leg Geometry Update")

    def update_gui_calculated_base_pos_display(self):
        # This method now safely reads values from spinboxes that might be disabled
        # or have special text, but their underlying .value() should be correct.
        self.calculated_base_positions = []
        fr_x = self.cfg_leg_front_corner_x.value();
        fr_y = self.cfg_leg_front_corner_y.value()
        mr_x = self.cfg_leg_middle_side_x.value()
        corner_ext = self.cfg_leg_corner_ext.value();
        middle_ext = self.cfg_leg_middle_ext.value()

        sym_base_xy = [
            (fr_x, -fr_y), (mr_x, 0.0), (fr_x, fr_y),  # Right side: BR, MR, FR
            (-fr_x, -fr_y), (-mr_x, 0.0), (-fr_x, fr_y)  # Left side: BL, ML, FL (as per LEG_NAMES order)
        ]

        # Ensure ESP_LEG_MOUNTING_ANGLES matches BR, MR, FR, BL, ML, FL if sym_base_xy is ordered this way
        # Current ESP_LEG_MOUNTING_ANGLES order: BR, MR, FR, BL, ML, FL - matches LEG_NAMES
        # Current sym_base_xy order seems to be:
        # Leg 0 (BR): (fr_x, -fr_y)
        # Leg 1 (MR): (mr_x, 0)
        # Leg 2 (FR): (fr_x, fr_y)
        # Leg 3 (BL): (-fr_x, -fr_y)
        # Leg 4 (ML): (-mr_x, 0)
        # Leg 5 (FL): (-fr_x, fr_y)
        # This mapping appears correct.

        for i in range(LEG_COUNT):
            base_x, base_y = sym_base_xy[i]
            # Determine if it's a corner leg (0, 2, 3, 5) or middle leg (1, 4)
            # Based on LEG_NAMES = ["BR", "MR", "FR", "BL", "ML", "FL"]
            # Corner legs: BR (0), FR (2), BL (3), FL (5)
            # Middle legs: MR (1), ML (4)
            is_middle_leg = (LEG_NAMES[i] == "MR" or LEG_NAMES[i] == "ML")  # or i == 1 or i == 4

            ext = middle_ext if is_middle_leg else corner_ext
            angle_rad = ESP_LEG_MOUNTING_ANGLES[i]

            final_x = base_x + ext * math.cos(angle_rad);
            final_y = base_y + ext * math.sin(angle_rad);
            final_z = 0.0  # Assuming base Z is 0 in this context

            self.calculated_base_positions.append({'x': final_x, 'y': final_y, 'z': final_z})
            self.gui_base_pos_labels[i]['x'].setText(f"X:{final_x:.1f}");
            self.gui_base_pos_labels[i]['y'].setText(f"Y:{final_y:.1f}");
            self.gui_base_pos_labels[i]['z'].setText(f"Z:{final_z:.1f}")

    @Slot()
    def send_pwm_reset_command_tcp(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            if self.comms_client.send_pwm_reinitialize_command():
                self.log_to_terminal("TX TCP: Sent PWM Re-initialize Command.")

    @Slot()
    def send_client_settings_to_robot(self):
        if not self.comms_client or not self.comms_client.is_tcp_connected():
            self.log_to_terminal("Cannot send client_settings: Not connected.");
            return
        try:
            gui_ip = self.comms_client.get_local_ip_for_telemetry_setup()
            if not gui_ip or gui_ip == "0.0.0.0":
                self.log_to_terminal(
                    "[ERROR] Could not determine valid local IP for telemetry. Cannot send client_settings.")
                return

            gui_udp_listen_port = DEFAULT_GUI_TELEMETRY_LISTEN_PORT

            udp_subs_payload = {
                "robot_state_actual": {
                    "enabled": self.sub_robot_state_actual_check.isChecked(),
                    "interval_ms": int(self.sub_robot_state_actual_interval.text())
                },
                "debug_foot_pos": {
                    "enabled": self.sub_debug_foot_pos_check.isChecked(),
                    "interval_ms": int(self.sub_debug_foot_pos_interval.text())
                }
            }
            tcp_subs_payload = {
                "battery": {
                    "enabled": self.sub_battery_check.isChecked(),
                    "interval_ms": int(self.sub_battery_interval.text())
                },
                "robot_status": {
                    "enabled": self.sub_robot_status_check.isChecked(),
                    "interval_ms": int(self.sub_robot_status_interval.text())
                }
            }
            final_settings_payload_refined = {
                "udp_telemetry_config": {
                    "target_ip": gui_ip,
                    "target_port": gui_udp_listen_port,
                    "subscriptions": udp_subs_payload
                },
                "tcp_subscriptions_here": tcp_subs_payload
            }
            if self.comms_client.send_client_settings(final_settings_payload_refined):
                self.log_to_terminal(f"TX TCP: Sent client_settings (UDP telemetry to {gui_ip}:{gui_udp_listen_port}).")

            # Update visibility of debug group based on new setting
            self.debug_foot_pos_group.setVisible(self.sub_debug_foot_pos_check.isChecked())

        except ValueError:
            self.log_to_terminal("[ERROR] Invalid interval for telemetry subscriptions. Must be integer.")
        except Exception as e:
            self.log_to_terminal(f"[ERROR] Sending client_settings: {e}")

    @Slot()
    def request_full_state_from_robot(self):
        if self.comms_client and self.comms_client.is_tcp_connected():
            gui_ip = self.comms_client.get_local_ip_for_telemetry_setup()
            if not gui_ip or gui_ip == "0.0.0.0":
                self.log_to_terminal(
                    "[ERROR] Could not determine valid local IP for telemetry. Cannot request full state.")
                return

            if self.comms_client.send_request_full_state(gui_ip):
                self.log_to_terminal(f"TX TCP: Requesting full state (reply to {gui_ip})")
        else:
            self.log_to_terminal("Cannot request full state: Not connected via TCP.")

    def closeEvent(self, event: QCloseEvent):
        self.log_to_terminal("Close event: Shutting down...")
        self.udp_intent_timer.stop()
        self.udp_telemetry_receiver.stop_listening()  # Stop UDP telemetry listener

        if self.comms_client:
            if self.comms_client.is_tcp_connected():
                self.comms_client.send_disconnect_notice();
                time.sleep(0.1)  # Give a moment for the notice
            self.comms_client.close()  # This handles TCP socket and thread in comms_client
            self.comms_client = None

        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HexapodControllerGUI()
    window.show()
    sys.exit(app.exec())