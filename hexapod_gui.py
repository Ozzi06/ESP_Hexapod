#!/usr/bin/env python3
import sys
import math
import time
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QTextEdit, QGroupBox, QFormLayout
)
from PySide6.QtCore import Qt, QTimer, Slot, QElapsedTimer
from PySide6.QtGui import QKeyEvent, QCloseEvent, QDoubleValidator

# Import the UDP client class
try:
    from hexapod_udp_client import HexapodUDPClient
except ImportError:
    print("Error: Could not find hexapod_udp_client.py.")
    print("Please make sure it's in the same directory.")
    sys.exit(1)

# --- Configuration ---
DEFAULT_TARGET_PORT = 5005
UPDATE_INTERVAL_MS = 100 # Target 10 Hz update rate
DEFAULT_LINEAR_SPEED = 5.0 # cm/s for WASD
DEFAULT_YAW_RATE = 0.5     # rad/s for QE
DEFAULT_POSE_ADJUST_SPEED_LINEAR = 2.0 # cm/s for Arrow Keys
DEFAULT_POSE_ADJUST_SPEED_ANGULAR = math.radians(15.0) # rad/s (~15 deg/s) for IJKL/UO

# --- Helper Quaternion Class (Simplified) ---
class Quaternion:
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
        # Quaternion multiplication (self * other)
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return Quaternion(w, x, y, z) # Normalization happens after use if needed

    @classmethod
    def from_axis_angle(cls, axis_x, axis_y, axis_z, angle_rad):
        half_angle = angle_rad / 2.0
        s = math.sin(half_angle)
        # Normalize axis vector first
        axis_mag = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        if axis_mag < 1e-9: return cls() # Return identity if axis is zero
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
        self.setGeometry(100, 100, 650, 550) # Adjusted size

        # UDP Client Instance
        self.hexapod_client = None # Initialized on connect

        # Timers
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_control_loop)
        self.frame_timer = QElapsedTimer() # For calculating dt

        # Input State
        self.keys_pressed = set()
        self.target_linear_speed = DEFAULT_LINEAR_SPEED
        self.target_yaw_rate = DEFAULT_YAW_RATE
        self.pose_adjust_speed_linear = DEFAULT_POSE_ADJUST_SPEED_LINEAR
        self.pose_adjust_speed_angular = DEFAULT_POSE_ADJUST_SPEED_ANGULAR
        self.acceleration_factor = 5.0 # How quickly speed ramps up/down (higher is faster)

        # Current Smoothed State (sent to robot)
        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
        self.current_yaw_rate = 0.0
        # Pose is updated directly but changes applied over time based on key presses
        self.current_body_pos = [0.0, 0.0, 10.0] # x, y, z offset
        self.current_body_orient = Quaternion(1.0, 0.0, 0.0, 0.0) # w, x, y, z

        self.last_update_time = 0

        self._init_ui()
        self.update_display() # Initialize display

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Connection Group ---
        connection_group = QGroupBox("Connection")
        connection_layout = QHBoxLayout()
        connection_group.setLayout(connection_layout)
        # No IP input needed for broadcast
        connection_layout.addWidget(QLabel("Port:"))
        self.port_input = QLineEdit(str(DEFAULT_TARGET_PORT))
        self.port_input.setValidator(QDoubleValidator(1024, 65535, 0)) # Port range validation
        self.port_input.setFixedWidth(60)
        connection_layout.addWidget(self.port_input)
        self.connect_button = QPushButton("Connect (Broadcast)")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_button)
        self.connection_status_label = QLabel("Status: Disconnected")
        connection_layout.addWidget(self.connection_status_label)
        connection_layout.addStretch()
        main_layout.addWidget(connection_group)

        # --- Control & Display Layout ---
        control_display_layout = QHBoxLayout()

        # Left Side: Parameters & Instructions
        left_layout = QVBoxLayout()

        # Target Speeds Input
        speeds_group = QGroupBox("Target Speeds")
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


        # Gait Parameters Input
        gait_group = QGroupBox("Gait Parameters")
        gait_form_layout = QFormLayout(gait_group)
        self.step_height_input = QDoubleSpinBox()
        self.step_height_input.setRange(0.0, 10.0); self.step_height_input.setDecimals(1); self.step_height_input.setSingleStep(0.1)
        self.step_height_input.setValue(3.0)
        gait_form_layout.addRow("Step Height (cm):", self.step_height_input)

        self.step_freq_input = QDoubleSpinBox()
        self.step_freq_input.setRange(0.1, 5.0); self.step_freq_input.setDecimals(1); self.step_freq_input.setSingleStep(0.1)
        self.step_freq_input.setValue(1.5)
        gait_form_layout.addRow("Step Frequency (Hz):", self.step_freq_input)

        self.duty_factor_input = QDoubleSpinBox()
        self.duty_factor_input.setRange(0.01, 0.99); self.duty_factor_input.setDecimals(2); self.duty_factor_input.setSingleStep(0.05)
        self.duty_factor_input.setValue(0.5)
        gait_form_layout.addRow("Duty Factor:", self.duty_factor_input)
        left_layout.addWidget(gait_group)

        # Instructions
        instructions_group = QGroupBox("Keyboard Controls")
        instructions_layout = QVBoxLayout(instructions_group)
        instructions_text = (
            "W/S: Forward/Backward (+/- Y Vel)\n"
            "A/D: Strafe Left/Right (+/- X Vel)\n"
            "Q/E: Turn Left/Right (+/- Yaw Rate)\n"
            "--- Body Pose Adjustment ---\n"
            "Up/Down Arrow: Move Forward/Backward (+/- Y Offset)\n"
            "Left/Right Arrow: Move Left/Right (+/- X Offset)\n"
            "Shift+Up/Down: Move Up/Down (+/- Z Offset)\n"
            "I/K: Pitch Down/Up (+/- Roll around X)\n"
            "J/L: Roll Left/Right (+/- Roll around Y)\n"
            "U/O: Yaw Left/Right (+/- Yaw around Z)\n"
             "SPACE: Toggle Walk/Stop"
        )
        instructions_label = QLabel(instructions_text)
        instructions_label.setWordWrap(True) # Ensure text wraps
        instructions_layout.addWidget(instructions_label)
        left_layout.addWidget(instructions_group)
        left_layout.addStretch()

        control_display_layout.addLayout(left_layout, 1) # Left side takes 1 part stretch

        # Right Side: Current State Display
        right_layout = QVBoxLayout()
        state_group = QGroupBox("Current Target State (Sent to Robot)")
        state_form_layout = QFormLayout(state_group)

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


        right_layout.addWidget(state_group)
        right_layout.addStretch()

        control_display_layout.addLayout(right_layout, 1) # Right side takes 1 part stretch


        main_layout.addLayout(control_display_layout)

        # Set focus policy to capture keys
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # --- Connection Handling ---
    @Slot()
    def toggle_connection(self):
        if self.hexapod_client is None:
            try:
                port = int(self.port_input.text())
                # Instantiate client for broadcast
                self.hexapod_client = HexapodUDPClient(target_ip="255.255.255.255",target_port=port)

                self.connection_status_label.setText("Status: Connected (Broadcast)")
                self.connection_status_label.setStyleSheet("color: green")
                self.connect_button.setText("Disconnect")
                self.port_input.setEnabled(False)
                self.update_timer.start(UPDATE_INTERVAL_MS)
                self.frame_timer.start()
                self.last_update_time = self.frame_timer.elapsed()
                print("Connected.")
            except ValueError:
                self.connection_status_label.setText("Status: Invalid Port")
                self.connection_status_label.setStyleSheet("color: red")
            except Exception as e:
                self.connection_status_label.setText(f"Status: Error - {e}")
                self.connection_status_label.setStyleSheet("color: red")
                if self.hexapod_client:
                    self.hexapod_client.close()
                self.hexapod_client = None
        else:
            print("Disconnecting...")
            self.update_timer.stop()
            if self.hexapod_client:
                 # Send final stop command
                try:
                    self.hexapod_client.set_walk_command(run=False)
                    self.hexapod_client.set_velocity(0, 0, 0)
                    self.hexapod_client.set_angular_velocity(0)
                    self.hexapod_client.send()
                    time.sleep(0.05) # Allow time to send
                    self.hexapod_client.close()
                except Exception as e:
                    print(f"Error sending final stop command: {e}")
            self.hexapod_client = None
            self.connection_status_label.setText("Status: Disconnected")
            self.connection_status_label.setStyleSheet("") # Reset color
            self.connect_button.setText("Connect (Broadcast)")
            self.port_input.setEnabled(True)
            # Reset current state for safety? Optional.
            # self.current_velocity_x = 0.0 ...


    # --- Event Handling ---
    def keyPressEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.keys_pressed.add(event.key())
            # Handle immediate actions like toggle walk
            if event.key() == Qt.Key.Key_Space:
                if self.hexapod_client:
                    self.hexapod_client.set_walk_command(not self.hexapod_client._walk_running)
                    print(f"Walk Toggled: {'Running' if self.hexapod_client._walk_running else 'Stopped'}")


    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            try:
                self.keys_pressed.remove(event.key())
            except KeyError:
                pass # Ignore if key wasn't in the set

    def closeEvent(self, event: QCloseEvent):
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
            except Exception as e:
                print(f"Error during final send/close: {e}")
        event.accept()

    # --- Control Loop ---
    @Slot()
    def update_control_loop(self):
        if self.hexapod_client is None:
            return

        current_time = self.frame_timer.elapsed()
        dt = (current_time - self.last_update_time) / 1000.0 # Delta time in seconds
        self.last_update_time = current_time
        if dt <= 0: dt = UPDATE_INTERVAL_MS / 1000.0 # Avoid zero/negative dt on first run or glitches

        # --- Calculate Target Velocities based on keys ---
        target_vx = 0.0
        target_vy = 0.0
        target_yaw = 0.0

        if Qt.Key.Key_W in self.keys_pressed: target_vy += self.target_linear_speed
        if Qt.Key.Key_S in self.keys_pressed: target_vy -= self.target_linear_speed
        if Qt.Key.Key_A in self.keys_pressed: target_vx -= self.target_linear_speed # Strafe Left
        if Qt.Key.Key_D in self.keys_pressed: target_vx += self.target_linear_speed # Strafe Right
        if Qt.Key.Key_Q in self.keys_pressed: target_yaw += self.target_yaw_rate    # Turn Left
        if Qt.Key.Key_E in self.keys_pressed: target_yaw -= self.target_yaw_rate    # Turn Right

        # --- Smooth Velocity Ramping ---
        accel = self.acceleration_factor * dt
        self.current_velocity_x += max(-accel, min(accel, target_vx - self.current_velocity_x))
        self.current_velocity_y += max(-accel, min(accel, target_vy - self.current_velocity_y))
        self.current_yaw_rate   += max(-accel, min(accel, target_yaw - self.current_yaw_rate))

        # Clamp to max speeds (important for ramping down too)
        if abs(self.current_velocity_x) > self.target_linear_speed:
             self.current_velocity_x = math.copysign(self.target_linear_speed, self.current_velocity_x)
        if abs(self.current_velocity_y) > self.target_linear_speed:
             self.current_velocity_y = math.copysign(self.target_linear_speed, self.current_velocity_y)
        if abs(self.current_yaw_rate) > self.target_yaw_rate:
             self.current_yaw_rate = math.copysign(self.target_yaw_rate, self.current_yaw_rate)


        # --- Process Direct Pose Adjustments ---
        pos_delta_x = 0.0
        pos_delta_y = 0.0
        pos_delta_z = 0.0
        rot_pitch = 0.0 # Rotation around Body X
        rot_roll = 0.0  # Rotation around Body Y
        rot_yaw = 0.0   # Rotation around Body Z

        adjust_lin_step = self.pose_adjust_speed_linear * dt
        adjust_ang_step = self.pose_adjust_speed_angular * dt

        # Check modifier keys for Z offset
        modifiers = QApplication.keyboardModifiers()
        shift_pressed = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)
        # ctrl_pressed = bool(modifiers & Qt.KeyboardModifier.ControlModifier) # Could use Ctrl too

        if Qt.Key.Key_Up in self.keys_pressed:
            if shift_pressed: pos_delta_z += adjust_lin_step # Shift+Up = Body Up
            else:             pos_delta_y += adjust_lin_step # Up = Body Forward
        if Qt.Key.Key_Down in self.keys_pressed:
            if shift_pressed: pos_delta_z -= adjust_lin_step # Shift+Down = Body Down
            else:             pos_delta_y -= adjust_lin_step # Down = Body Backward
        if Qt.Key.Key_Left in self.keys_pressed:  pos_delta_x -= adjust_lin_step # Left = Body Left
        if Qt.Key.Key_Right in self.keys_pressed: pos_delta_x += adjust_lin_step # Right = Body Right

        # Orientation adjustments (relative to current body orientation)
        if Qt.Key.Key_I in self.keys_pressed: rot_pitch += adjust_ang_step # Pitch Down (Nose Down)
        if Qt.Key.Key_K in self.keys_pressed: rot_pitch -= adjust_ang_step # Pitch Up (Nose Up)
        if Qt.Key.Key_J in self.keys_pressed: rot_roll  += adjust_ang_step # Roll Left (relative to body)
        if Qt.Key.Key_L in self.keys_pressed: rot_roll  -= adjust_ang_step # Roll Right (relative to body)
        if Qt.Key.Key_U in self.keys_pressed: rot_yaw   += adjust_ang_step # Yaw Left (relative to body)
        if Qt.Key.Key_O in self.keys_pressed: rot_yaw   -= adjust_ang_step # Yaw Right (relative to body)

        # Apply position delta
        self.current_body_pos[0] += pos_delta_x
        self.current_body_pos[1] += pos_delta_y
        self.current_body_pos[2] += pos_delta_z

        # Apply orientation delta by composing rotations
        # IMPORTANT: Rotations are applied in Body Frame (relative to current orientation)
        # Order can matter slightly (Yaw, Pitch, Roll is common)
        if abs(rot_yaw) > 1e-6:
            rot_q = Quaternion.from_axis_angle(0, 0, 1, rot_yaw) # Rotate around Body Z
            self.current_body_orient = self.current_body_orient * rot_q # Post-multiply for body frame rotation
        if abs(rot_pitch) > 1e-6:
            rot_q = Quaternion.from_axis_angle(1, 0, 0, rot_pitch) # Rotate around Body X
            self.current_body_orient = self.current_body_orient * rot_q
        if abs(rot_roll) > 1e-6:
            rot_q = Quaternion.from_axis_angle(0, 1, 0, rot_roll) # Rotate around Body Y
            self.current_body_orient = self.current_body_orient * rot_q

        self.current_body_orient.normalize() # Ensure it stays normalized


        # --- Update Hexapod Client State ---
        self.hexapod_client.set_velocity(self.current_velocity_x, self.current_velocity_y) # Z velocity not used here
        self.hexapod_client.set_angular_velocity(self.current_yaw_rate)

        # Set gait params from UI inputs
        self.hexapod_client.set_gait_params(
            self.step_height_input.value(),
            self.step_freq_input.value(),
            self.duty_factor_input.value()
        )
        # Set body pose from current smoothed/adjusted state
        self.hexapod_client.set_body_position(
            self.current_body_pos[0], self.current_body_pos[1], self.current_body_pos[2]
        )
        orient_tuple = self.current_body_orient.to_tuple()
        self.hexapod_client.set_body_orientation(
            orient_tuple[0], orient_tuple[1], orient_tuple[2], orient_tuple[3], normalize=False # Already normalized
        )

        # --- Send Packet ---
        self.hexapod_client.send()

        # --- Update UI Display ---
        self.update_display()


    # --- UI Update ---
    def update_display(self):
        if self.hexapod_client is None:
             self.walk_status_label.setText("STOPPED (Disconnected)")
             # Optionally clear other labels or grey them out
             return

        # Display the state *being sent*
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
        self.gait_f_label.setText(f"{self.step_freq_input.value():.1f} Hz")
        self.gait_d_label.setText(f"{self.duty_factor_input.value():.2f}")

        self.walk_status_label.setText("RUNNING" if self.hexapod_client._walk_running else "STOPPED")
        self.walk_status_label.setStyleSheet("color: green" if self.hexapod_client._walk_running else "color: red")


# --- Main Execution ---
if __name__ == "__main__":
    print("Starting Hexapod GUI Controller...")
    print("Remember to run this script with Administrator privileges on Windows for broadcast.")
    app = QApplication(sys.argv)
    window = HexapodControllerGUI()
    window.show()
    sys.exit(app.exec())