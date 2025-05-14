#!/usr/bin/env python3
import sys
import math
import time
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QGroupBox, QFormLayout,
    QSizePolicy # ### Added for layout control ###
)
from PySide6.QtCore import Qt, QTimer, Slot, QElapsedTimer
from PySide6.QtGui import QKeyEvent, QCloseEvent, QDoubleValidator

try:
    from hexapod_udp_client import HexapodUDPClient, LEG_COUNT
except ImportError:
    print("Error: Could not find hexapod_udp_client.py or LEG_COUNT definition.")
    sys.exit(1)

# --- Configuration ---
DEFAULT_TARGET_PORT = 5005
DEFAULT_UPDATE_FREQUENCY_HZ = 50.0 # ### Default send rate ###
MIN_UPDATE_FREQUENCY_HZ = 1.0
MAX_UPDATE_FREQUENCY_HZ = 100.0

DEFAULT_LINEAR_SPEED = 8.0
DEFAULT_YAW_RATE = 0.3
DEFAULT_POSE_ADJUST_SPEED_LINEAR = 2.0
DEFAULT_POSE_ADJUST_SPEED_ANGULAR = math.radians(15.0)

# ### ROBOT SPEC CONSTANTS - MUST MATCH YOUR robot_spec.cpp ###
# Example values - replace with your actual robot's configuration
LEG_NAMES = ["BR", "MR", "FR", "BL", "ML", "FL"]
DEFAULT_BASE_POSITIONS = [ # (X, Y, Z) in Walk Frame
    [27.0, -19.0, 0.0], [32.0, 0.0, 0.0], [27.0, 19.0, 0.0], # Right side (0, 1, 2)
    [-27.0, -19.0, 0.0], [-32.0, 0.0, 0.0], [-27.0, 19.0, 0.0] # Left side (3, 4, 5)
]
# Angles (radians) assumed relative to Walk Frame +X (Right)
LEG_MOUNTING_ANGLES = [
    -1 * math.pi / 4.0,  0 * math.pi / 4.0, +1 * math.pi / 4.0, # Right side (0, 1, 2)
    -3 * math.pi / 4.0, +4 * math.pi / 4.0, +3 * math.pi / 4.0  # Left side (3, 4, 5) - Note: +4*pi/4 is just PI
]
# ### END ROBOT SPEC CONSTANTS ###


# ### Simplified Quaternion Class (Keep as before) ###
class Quaternion:
    # ... (Keep the exact same Quaternion class implementation) ...
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        self.normalize()

    def normalize(self):
        mag = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if mag > 1e-9:
            self.w /= mag
            self.x /= mag
            self.y /= mag
            self.z /= mag
        else:
            self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0 # Reset to identity
        return self

    def __mul__(self, other):
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return Quaternion(w, x, y, z)

    @classmethod
    def from_axis_angle(cls, axis_x, axis_y, axis_z, angle_rad):
        half_angle = angle_rad / 2.0
        s = math.sin(half_angle)
        axis_mag = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        if axis_mag < 1e-9: return cls()
        axis_x /= axis_mag
        axis_y /= axis_mag
        axis_z /= axis_mag
        return cls(math.cos(half_angle), axis_x * s, axis_y * s, axis_z * s)

    def to_tuple(self):
        return (self.w, self.x, self.y, self.z)

# --- Main GUI Window ---
class HexapodControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hexapod UDP Controller")
        self.setGeometry(100, 100, 750, 600) # ### Adjusted size ###

        self.hexapod_client = None
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_control_loop)
        self.frame_timer = QElapsedTimer()

        self.keys_pressed = set()
        self.target_linear_speed = DEFAULT_LINEAR_SPEED
        self.target_yaw_rate = DEFAULT_YAW_RATE
        self.pose_adjust_speed_linear = DEFAULT_POSE_ADJUST_SPEED_LINEAR
        self.pose_adjust_speed_angular = DEFAULT_POSE_ADJUST_SPEED_ANGULAR
        self.acceleration_factor = 5.0

        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
        self.current_yaw_rate = 0.0
        self.current_body_pos = [0.0, 0.0, 10.0]
        self.current_body_orient = Quaternion(1.0, 0.0, 0.0, 0.0)
        # ### Store current extension values ###
        self.current_leg_extensions = [0.0] * LEG_COUNT
        self._base_pos_needs_recalculation = True # Flag to trigger calculation

        self.last_update_time = 0

        self._init_ui()
        self.update_display()

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Connection Group (Added Frequency) ---
        connection_group = QGroupBox("Connection & Rate")
        connection_layout = QHBoxLayout(connection_group)
        connection_layout.addWidget(QLabel("Port:"))
        self.port_input = QLineEdit(str(DEFAULT_TARGET_PORT))
        self.port_input.setValidator(QDoubleValidator(1024, 65535, 0))
        self.port_input.setFixedWidth(60)
        connection_layout.addWidget(self.port_input)
        # ### Update Frequency Input ###
        connection_layout.addWidget(QLabel("Update Freq (Hz):"))
        self.update_freq_input = QDoubleSpinBox()
        self.update_freq_input.setRange(MIN_UPDATE_FREQUENCY_HZ, MAX_UPDATE_FREQUENCY_HZ)
        self.update_freq_input.setDecimals(1)
        self.update_freq_input.setSingleStep(1.0)
        self.update_freq_input.setValue(DEFAULT_UPDATE_FREQUENCY_HZ)
        self.update_freq_input.valueChanged.connect(self.update_timer_interval) # ### Connect signal ###
        connection_layout.addWidget(self.update_freq_input)

        self.connect_button = QPushButton("Connect (Broadcast)")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_button)
        self.connection_status_label = QLabel("Status: Disconnected")
        connection_layout.addWidget(self.connection_status_label)
        connection_layout.addStretch()
        main_layout.addWidget(connection_group)

        # --- Control & Display Layout ---
        control_display_layout = QHBoxLayout()

        # --- Left Side: Parameters & Inputs ---
        left_widget = QWidget() # ### Use a container widget ###
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0,0,0,0) # Remove margins if needed

        # Target Speeds Input (Same as before)
        speeds_group = QGroupBox("Target Speeds")
        # ... (Keep the speeds group setup exactly as before) ...
        speeds_form_layout = QFormLayout(speeds_group)
        self.target_linear_speed_input = QDoubleSpinBox()
        self.target_linear_speed_input.setRange(0, 50.0); self.target_linear_speed_input.setDecimals(1); self.target_linear_speed_input.setSingleStep(0.5)
        self.target_linear_speed_input.setValue(DEFAULT_LINEAR_SPEED)
        self.target_linear_speed_input.valueChanged.connect(lambda v: setattr(self, 'target_linear_speed', v))
        speeds_form_layout.addRow("Linear Vel (cm/s, WASD):", self.target_linear_speed_input)
        self.target_yaw_rate_input = QDoubleSpinBox()
        self.target_yaw_rate_input.setRange(0, math.pi); self.target_yaw_rate_input.setDecimals(2); self.target_yaw_rate_input.setSingleStep(0.1)
        self.target_yaw_rate_input.setValue(DEFAULT_YAW_RATE)
        self.target_yaw_rate_input.valueChanged.connect(lambda v: setattr(self, 'target_yaw_rate', v))
        speeds_form_layout.addRow("Yaw Rate (rad/s, QE):", self.target_yaw_rate_input)
        self.pose_linear_speed_input = QDoubleSpinBox()
        self.pose_linear_speed_input.setRange(0, 10.0); self.pose_linear_speed_input.setDecimals(1); self.pose_linear_speed_input.setSingleStep(0.1)
        self.pose_linear_speed_input.setValue(DEFAULT_POSE_ADJUST_SPEED_LINEAR)
        self.pose_linear_speed_input.valueChanged.connect(lambda v: setattr(self, 'pose_adjust_speed_linear', v))
        speeds_form_layout.addRow("Pose Lin Spd (cm/s, Arws):", self.pose_linear_speed_input)
        self.pose_angular_speed_input = QDoubleSpinBox()
        self.pose_angular_speed_input.setRange(0, 90.0); self.pose_angular_speed_input.setDecimals(1); self.pose_angular_speed_input.setSingleStep(1.0)
        self.pose_angular_speed_input.setValue(math.degrees(DEFAULT_POSE_ADJUST_SPEED_ANGULAR)) # Display in degrees
        self.pose_angular_speed_input.valueChanged.connect(lambda v: setattr(self, 'pose_adjust_speed_angular', math.radians(v)))
        speeds_form_layout.addRow("Pose Ang Spd (deg/s, IJKLUO):", self.pose_angular_speed_input)
        left_layout.addWidget(speeds_group)

        # Gait Parameters Input (Same as before)
        gait_group = QGroupBox("Gait Parameters")
        # ... (Keep the gait group setup exactly as before) ...
        gait_form_layout = QFormLayout(gait_group)
        self.step_height_input = QDoubleSpinBox()
        self.step_height_input.setRange(0.0, 10.0); self.step_height_input.setDecimals(1); self.step_height_input.setSingleStep(0.1)
        self.step_height_input.setValue(3.0)
        gait_form_layout.addRow("Step Height (cm):", self.step_height_input)
        self.step_time_input = QDoubleSpinBox()
        self.step_time_input.setRange(0.1, 5.0); self.step_time_input.setDecimals(1); self.step_time_input.setSingleStep(0.1)
        self.step_time_input.setValue(1.5)
        gait_form_layout.addRow("Step Time (s):", self.step_time_input)
        self.duty_factor_input = QDoubleSpinBox()
        self.duty_factor_input.setRange(0.01, 0.99); self.duty_factor_input.setDecimals(2); self.duty_factor_input.setSingleStep(0.05)
        self.duty_factor_input.setValue(0.5)
        gait_form_layout.addRow("Duty Factor:", self.duty_factor_input)
        left_layout.addWidget(gait_group)

        # Instructions (Same as before)
        instructions_group = QGroupBox("Keyboard Controls")
        # ... (Keep the instructions group setup exactly as before) ...
        instructions_layout = QVBoxLayout(instructions_group)
        instructions_text = (
            "W/S: Forward/Backward\nA/D: Strafe Left/Right\n"
            "Q/E: Turn Left/Right\nSPACE: Toggle Walk/Stop\n"
            "--- Body Pose ---\nArrows: Move Body XY\nShift+Arrows: Move Body Z\n"
            "I/K: Pitch\nJ/L: Roll\nU/O: Yaw\n"
            "--- Leg Extensions (See Controls Below) ---" # ### Updated instructions ###
        )
        instructions_label = QLabel(instructions_text)
        instructions_label.setWordWrap(True)
        instructions_layout.addWidget(instructions_label)
        left_layout.addWidget(instructions_group)

        left_layout.addStretch()
        control_display_layout.addWidget(left_widget, 1) # Add container widget

        # --- Center: Leg Extension Controls --- ### NEW SECTION ###
        extension_widget = QWidget()
        extension_layout = QVBoxLayout(extension_widget)
        extension_layout.setContentsMargins(0,0,0,0)

        extension_group = QGroupBox("Leg Extensions (Horizontal Outward, cm)")
        extension_form_layout = QFormLayout(extension_group)
        self.extension_inputs = []
        for i in range(LEG_COUNT):
            ext_input = QDoubleSpinBox()
            ext_input.setRange(-5.0, 10.0) # Allow contraction and extension
            ext_input.setDecimals(1)
            ext_input.setSingleStep(0.1)
            ext_input.setValue(0.0) # Default to no extension
            ext_input.valueChanged.connect(self.mark_base_pos_recalc_needed) # ### Connect signal ###
            self.extension_inputs.append(ext_input)
            extension_form_layout.addRow(f"Leg {i} ({LEG_NAMES[i]}):", ext_input)

        reset_ext_button = QPushButton("Reset All Extensions")
        reset_ext_button.clicked.connect(self.reset_extensions)
        extension_form_layout.addRow(reset_ext_button)

        extension_layout.addWidget(extension_group)
        extension_layout.addStretch()
        control_display_layout.addWidget(extension_widget, 1) # Add extension controls widget


        # --- Right Side: Current State Display ---
        right_widget = QWidget() # ### Use a container widget ###
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0,0,0,0)

        state_group = QGroupBox("Current Target State (Sent to Robot)")
        state_form_layout = QFormLayout(state_group)
        # ... (Keep all the labels self.velocity_x_label etc. as before) ...
        self.velocity_x_label = QLabel("0.00 cm/s")
        self.velocity_y_label = QLabel("0.00 cm/s")
        self.yaw_rate_label = QLabel("0.00 rad/s")
        state_form_layout.addRow("Target Velocity X:", self.velocity_x_label)
        state_form_layout.addRow("Target Velocity Y:", self.velocity_y_label)
        state_form_layout.addRow("Target Yaw Rate:", self.yaw_rate_label)
        self.pos_x_label = QLabel("0.00 cm")
        self.pos_y_label = QLabel("0.00 cm")
        self.pos_z_label = QLabel("10.00 cm")
        state_form_layout.addRow("Body Pos Offset X:", self.pos_x_label)
        state_form_layout.addRow("Body Pos Offset Y:", self.pos_y_label)
        state_form_layout.addRow("Body Pos Offset Z:", self.pos_z_label)
        self.orient_w_label = QLabel("1.000")
        self.orient_x_label = QLabel("0.000")
        self.orient_y_label = QLabel("0.000")
        self.orient_z_label = QLabel("0.000")
        state_form_layout.addRow("Body Orient W:", self.orient_w_label)
        state_form_layout.addRow("Body Orient X:", self.orient_x_label)
        state_form_layout.addRow("Body Orient Y:", self.orient_y_label)
        state_form_layout.addRow("Body Orient Z:", self.orient_z_label)
        self.gait_h_label = QLabel("3.0 cm")
        self.gait_f_label = QLabel("1.5 Hz")
        self.gait_d_label = QLabel("0.50")
        state_form_layout.addRow("Gait Height:", self.gait_h_label)
        state_form_layout.addRow("Gait Freq:", self.gait_f_label)
        state_form_layout.addRow("Gait Duty:", self.gait_d_label)
        self.walk_status_label = QLabel("STOPPED")
        state_form_layout.addRow("Walk Status:", self.walk_status_label)
        # ### Add Base Position Display (Optional but good for debugging) ###
        self.base_pos_labels = []
        for i in range(LEG_COUNT):
             row_layout = QHBoxLayout()
             label_x = QLabel("X: 0.0")
             label_y = QLabel("Y: 0.0")
             label_z = QLabel("Z: 0.0")
             row_layout.addWidget(label_x)
             row_layout.addWidget(label_y)
             row_layout.addWidget(label_z)
             self.base_pos_labels.append({'x': label_x, 'y': label_y, 'z': label_z})
             state_form_layout.addRow(f"Base Pos {i} ({LEG_NAMES[i]}):", row_layout)


        right_layout.addWidget(state_group)
        right_layout.addStretch()
        control_display_layout.addWidget(right_widget, 1)

        main_layout.addLayout(control_display_layout)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # --- Connection Handling (Updated for Frequency) ---
    @Slot()
    def toggle_connection(self):
        if self.hexapod_client is None:
            try:
                port = int(self.port_input.text())
                freq_hz = self.update_freq_input.value()
                if freq_hz < MIN_UPDATE_FREQUENCY_HZ: # Basic validation
                    raise ValueError("Frequency too low")

                # ### Pass initial base positions to client ###
                self.hexapod_client = HexapodUDPClient(port, use_broadcast=True,
                                                       initial_base_pos=DEFAULT_BASE_POSITIONS)
                # ### Force recalculation on connect ###
                self.recalculate_all_base_positions()

                self.connection_status_label.setText("Status: Connected (Broadcast)")
                self.connection_status_label.setStyleSheet("color: green")
                self.connect_button.setText("Disconnect")
                self.port_input.setEnabled(False)
                self.update_freq_input.setEnabled(False) # ### Disable while connected ###

                interval_ms = int(1000.0 / freq_hz)
                self.update_timer.start(interval_ms)
                self.frame_timer.start()
                self.last_update_time = self.frame_timer.elapsed()
                print(f"Connected. Update interval: {interval_ms} ms ({freq_hz:.1f} Hz)")
            except ValueError as ve:
                 self.connection_status_label.setText(f"Status: Invalid Input ({ve})")
                 self.connection_status_label.setStyleSheet("color: red")
                 self.hexapod_client = None # Ensure client is None
            except Exception as e:
                self.connection_status_label.setText(f"Status: Error - {e}")
                self.connection_status_label.setStyleSheet("color: red")
                if self.hexapod_client: self.hexapod_client.close()
                self.hexapod_client = None
        else:
            # ... (Disconnect logic remains mostly the same) ...
            print("Disconnecting...")
            self.update_timer.stop()
            if self.hexapod_client:
                try: # Send final stop
                    self.hexapod_client.set_walk_command(run=False)
                    self.hexapod_client.set_velocity(0, 0, 0)
                    self.hexapod_client.set_angular_velocity(0)
                    self.hexapod_client.send()
                    time.sleep(0.05)
                    self.hexapod_client.close()
                except Exception as e: print(f"Error sending final stop: {e}")
            self.hexapod_client = None
            self.connection_status_label.setText("Status: Disconnected")
            self.connection_status_label.setStyleSheet("")
            self.connect_button.setText("Connect (Broadcast)")
            self.port_input.setEnabled(True)
            self.update_freq_input.setEnabled(True) # ### Re-enable frequency input ###


    # ### Slot to handle frequency changes ###
    @Slot()
    def update_timer_interval(self):
         # This is called when the frequency input changes value
         # We only adjust the timer IF already connected
         if self.hexapod_client is not None and self.update_timer.isActive():
            try:
                freq_hz = self.update_freq_input.value()
                if freq_hz < MIN_UPDATE_FREQUENCY_HZ:
                     print("[Warning] Frequency set too low, using minimum.")
                     freq_hz = MIN_UPDATE_FREQUENCY_HZ
                     self.update_freq_input.setValue(freq_hz) # Update UI

                interval_ms = int(1000.0 / freq_hz)
                self.update_timer.setInterval(interval_ms)
                print(f"Update interval changed to: {interval_ms} ms ({freq_hz:.1f} Hz)")
            except ValueError:
                print("[Error] Invalid frequency value during update.")
            except ZeroDivisionError:
                 print("[Error] Frequency cannot be zero.")


    # --- Event Handling (keyPress/Release, closeEvent are same) ---
    def keyPressEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.keys_pressed.add(event.key())
            if event.key() == Qt.Key.Key_Space:
                if self.hexapod_client:
                    self.hexapod_client.set_walk_command(not self.hexapod_client._walk_running)
                    print(f"Walk Toggled: {'Running' if self.hexapod_client._walk_running else 'Stopped'}")

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            try: self.keys_pressed.remove(event.key())
            except KeyError: pass

    def closeEvent(self, event: QCloseEvent):
        # ... (Keep closeEvent logic exactly as before) ...
        print("Close event triggered.")
        self.update_timer.stop()
        if self.hexapod_client:
            print("Sending final stop command before closing...")
            try:
                self.hexapod_client.set_walk_command(run=False)
                self.hexapod_client.set_velocity(0, 0, 0)
                self.hexapod_client.set_angular_velocity(0)
                self.hexapod_client.send()
                time.sleep(0.05)
                self.hexapod_client.close()
            except Exception as e: print(f"Error during final send/close: {e}")
        event.accept()


    # --- Base Position Calculation --- ### NEW SECTION ###
    @Slot()
    def mark_base_pos_recalc_needed(self):
        """Flags that base positions need recalculation in the next loop."""
        self._base_pos_needs_recalculation = True

    def recalculate_all_base_positions(self):
        """Calculates target XYZ for all legs based on extension inputs."""
        if self.hexapod_client is None: return

        # print("Recalculating base positions...") # Debug print
        for i in range(LEG_COUNT):
            try:
                extension = self.extension_inputs[i].value()
                default_pos = DEFAULT_BASE_POSITIONS[i]
                angle_rad = LEG_MOUNTING_ANGLES[i]

                # Calculate direction vector (outward, relative to Walk Frame +X)
                dir_x = math.cos(angle_rad)
                dir_y = math.sin(angle_rad)

                # Calculate new position
                new_x = default_pos[0] + extension * dir_x
                new_y = default_pos[1] + extension * dir_y
                new_z = default_pos[2] # Keep original Z

                # Update the client's internal state
                self.hexapod_client.set_base_foot_position(i, new_x, new_y, new_z)

            except IndexError:
                print(f"[Error] Index out of bounds during base position calculation for leg {i}.")
            except Exception as e:
                 print(f"[Error] Unexpected error calculating base pos for leg {i}: {e}")

        self._base_pos_needs_recalculation = False # Mark as done for this cycle

    @Slot()
    def reset_extensions(self):
        """Resets all leg extension inputs to 0."""
        print("Resetting leg extensions.")
        for i in range(LEG_COUNT):
            self.extension_inputs[i].setValue(0.0)
        # Recalculation will happen automatically due to valueChanged signal


    # --- Control Loop (Updated) ---
    @Slot()
    def update_control_loop(self):
        if self.hexapod_client is None: return

        current_time = self.frame_timer.elapsed()
        dt = (current_time - self.last_update_time) / 1000.0
        self.last_update_time = current_time
        if dt <= 0: dt = (1000.0 / self.update_freq_input.value()) / 1000.0 # Use configured interval if dt fails

        # --- Recalculate Base Positions if needed --- ### NEW STEP ###
        if self._base_pos_needs_recalculation:
             self.recalculate_all_base_positions()

        # --- Calculate Target Velocities (Same as before) ---
        # ... (Keep velocity calculation and smoothing logic exactly as before) ...
        target_vx = 0.0
        target_vy = 0.0
        target_yaw = 0.0
        if Qt.Key.Key_W in self.keys_pressed: target_vy += self.target_linear_speed
        if Qt.Key.Key_S in self.keys_pressed: target_vy -= self.target_linear_speed
        if Qt.Key.Key_A in self.keys_pressed: target_vx += self.target_linear_speed
        if Qt.Key.Key_D in self.keys_pressed: target_vx -= self.target_linear_speed
        if Qt.Key.Key_Q in self.keys_pressed: target_yaw -= self.target_yaw_rate
        if Qt.Key.Key_E in self.keys_pressed: target_yaw += self.target_yaw_rate
        accel = self.acceleration_factor * dt
        self.current_velocity_x += max(-accel, min(accel, target_vx - self.current_velocity_x))
        self.current_velocity_y += max(-accel, min(accel, target_vy - self.current_velocity_y))
        self.current_yaw_rate   += max(-accel, min(accel, target_yaw - self.current_yaw_rate))
        # Clamping (same as before)
        if abs(self.current_velocity_x) > self.target_linear_speed: self.current_velocity_x = math.copysign(self.target_linear_speed, self.current_velocity_x)
        if abs(self.current_velocity_y) > self.target_linear_speed: self.current_velocity_y = math.copysign(self.target_linear_speed, self.current_velocity_y)
        if abs(self.current_yaw_rate) > self.target_yaw_rate: self.current_yaw_rate = math.copysign(self.target_yaw_rate, self.current_yaw_rate)

        # --- Process Direct Pose Adjustments (Same as before) ---
        # ... (Keep pose adjustment logic exactly as before) ...
        pos_delta_x = 0.0; pos_delta_y = 0.0; pos_delta_z = 0.0
        rot_pitch = 0.0; rot_roll = 0.0; rot_yaw = 0.0
        adjust_lin_step = self.pose_adjust_speed_linear * dt
        adjust_ang_step = self.pose_adjust_speed_angular * dt
        modifiers = QApplication.keyboardModifiers()
        shift_pressed = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)
        if Qt.Key.Key_Up in self.keys_pressed:    pos_delta_y += adjust_lin_step if not shift_pressed else 0; pos_delta_z += adjust_lin_step if shift_pressed else 0
        if Qt.Key.Key_Down in self.keys_pressed:  pos_delta_y -= adjust_lin_step if not shift_pressed else 0; pos_delta_z -= adjust_lin_step if shift_pressed else 0
        if Qt.Key.Key_Left in self.keys_pressed:  pos_delta_x += adjust_lin_step
        if Qt.Key.Key_Right in self.keys_pressed: pos_delta_x -= adjust_lin_step
        if Qt.Key.Key_I in self.keys_pressed: rot_pitch -= adjust_ang_step
        if Qt.Key.Key_K in self.keys_pressed: rot_pitch += adjust_ang_step
        if Qt.Key.Key_J in self.keys_pressed: rot_roll  += adjust_ang_step
        if Qt.Key.Key_L in self.keys_pressed: rot_roll  -= adjust_ang_step
        if Qt.Key.Key_U in self.keys_pressed: rot_yaw   -= adjust_ang_step
        if Qt.Key.Key_O in self.keys_pressed: rot_yaw   += adjust_ang_step
        self.current_body_pos[0] += pos_delta_x
        self.current_body_pos[1] += pos_delta_y
        self.current_body_pos[2] += pos_delta_z
        if abs(rot_yaw) > 1e-6:   self.current_body_orient = self.current_body_orient * Quaternion.from_axis_angle(0, 0, 1, rot_yaw)
        if abs(rot_pitch) > 1e-6: self.current_body_orient = self.current_body_orient * Quaternion.from_axis_angle(1, 0, 0, rot_pitch)
        if abs(rot_roll) > 1e-6:  self.current_body_orient = self.current_body_orient * Quaternion.from_axis_angle(0, 1, 0, rot_roll)
        self.current_body_orient.normalize()

        # --- Update Hexapod Client State (Base Pos already updated if needed) ---
        self.hexapod_client.set_velocity(self.current_velocity_x, self.current_velocity_y)
        self.hexapod_client.set_angular_velocity(self.current_yaw_rate)
        self.hexapod_client.set_gait_params(
            self.step_height_input.value(), self.step_time_input.value(), self.duty_factor_input.value()
        )
        self.hexapod_client.set_body_position(
            self.current_body_pos[0], self.current_body_pos[1], self.current_body_pos[2]
        )
        orient_tuple = self.current_body_orient.to_tuple()
        self.hexapod_client.set_body_orientation(
            orient_tuple[0], orient_tuple[1], orient_tuple[2], orient_tuple[3], normalize=False
        )

        # --- Send Packet ---
        self.hexapod_client.send()

        # --- Update UI Display ---
        self.update_display()


    # --- UI Update (Added Base Pos Display) ---
    def update_display(self):
        if self.hexapod_client is None:
             self.walk_status_label.setText("STOPPED (Disconnected)")
             return

        # ... (Update velocity, pose, gait labels as before) ...
        self.velocity_x_label.setText(f"{self.current_velocity_x:.2f} cm/s")
        self.velocity_y_label.setText(f"{self.current_velocity_y:.2f} cm/s")
        self.yaw_rate_label.setText(f"{self.current_yaw_rate:.3f} rad/s")
        self.pos_x_label.setText(f"{self.current_body_pos[0]:.2f} cm")
        self.pos_y_label.setText(f"{self.current_body_pos[1]:.2f} cm")
        self.pos_z_label.setText(f"{self.current_body_pos[2]:.2f} cm")
        orient_tuple = self.current_body_orient.to_tuple()
        self.orient_w_label.setText(f"{orient_tuple[0]:.3f}")
        self.orient_x_label.setText(f"{orient_tuple[1]:.3f}")
        self.orient_y_label.setText(f"{orient_tuple[2]:.3f}")
        self.orient_z_label.setText(f"{orient_tuple[3]:.3f}")
        self.gait_h_label.setText(f"{self.step_height_input.value():.1f} cm")
        self.gait_f_label.setText(f"{self.step_time_input.value():.1f} Hz")
        self.gait_d_label.setText(f"{self.duty_factor_input.value():.2f}")
        self.walk_status_label.setText("RUNNING" if self.hexapod_client._walk_running else "STOPPED")
        self.walk_status_label.setStyleSheet("color: green" if self.hexapod_client._walk_running else "color: red")

        # ### Update base position display labels ###
        if hasattr(self.hexapod_client, 'base_foot_pos'):
            for i in range(LEG_COUNT):
                try:
                    pos = self.hexapod_client.base_foot_pos[i]
                    self.base_pos_labels[i]['x'].setText(f"X: {pos[0]:.1f}")
                    self.base_pos_labels[i]['y'].setText(f"Y: {pos[1]:.1f}")
                    self.base_pos_labels[i]['z'].setText(f"Z: {pos[2]:.1f}")
                except (IndexError, KeyError):
                    pass # Handle potential errors if labels/data mismatch


# --- Main Execution ---
if __name__ == "__main__":
    # ... (Keep main execution block as before) ...
    print("Starting Hexapod GUI Controller...")
    print("Remember to run this script with Administrator privileges on Windows for broadcast.")
    app = QApplication(sys.argv)
    # ### Ensure LEG_COUNT matches between GUI and Client ###
    if LEG_COUNT != len(LEG_NAMES) or LEG_COUNT != len(DEFAULT_BASE_POSITIONS) or LEG_COUNT != len(LEG_MOUNTING_ANGLES):
         print("[FATAL ERROR] Mismatch in LEG_COUNT definitions!")
         sys.exit(1)
    window = HexapodControllerGUI()
    window.show()
    sys.exit(app.exec())