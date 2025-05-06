# ESP32-S3 Hexapod Control Firmware

## Project Goal

Develop a versatile C++ firmware for an 18-DOF hexapod robot using an ESP32-S3 microcontroller. This project enables remote control via UDP/WiFi, featuring a layered kinematic system that allows independent control over the robot body's pose (position and orientation) relative to its base locomotion path.

## Hardware

*   **Microcontroller:** Xiao ESP32-S3
*   **Servo Motors:** 18x High-Torque Hobby Servos (e.g., 35kg-cm), 3 per leg (Coxa, Femur, Tibia).
*   **Servo Driver:** 2x Adafruit PCA9685 16-Channel PWM Servo Driver boards (connected via I2C).
*   **Connectivity:** WiFi (for UDP remote control).
*   **(Optional):** PC or other device for sending UDP control packets (Python GUI controller provided: `hexapod_controller_gui.py`).

## Software Architecture

The firmware utilizes a state machine (`hexapod_proj.ino`) to switch between operating modes. The core of the remote control and walking logic relies on a layered kinematic approach with distinct coordinate frames:

1.  **State Machine (`hexapod_proj.ino`):** Manages overall program flow and mode selection via Serial Monitor.
2.  **Coordinate Frames & Kinematics:** (See details below)
    *   **Walk Frame:** The primary reference frame for locomotion.
    *   **Body Frame:** Attached to the robot's physical chassis.
    *   **Leg Frame:** Local frame for each leg's IK calculation.
    *   **Walk Cycle (`walkcycle.h`):** Generates target foot positions within the **Walk Frame**.
    *   **Body Transformation (`body_transform.h`):** Transforms targets from the **Walk Frame** to the **Leg Frame**, incorporating desired body pose.
    *   **Inverse Kinematics (`ik.h`):** Calculates joint angles needed to reach the target in the **Leg Frame**.
3.  **Low-Level Control (`utils.h`):** Handles I2C communication with PCA9685 drivers and converts angles to PWM signals.
4.  **Math Utilities (`Vec3.h`, `quat.h`):** Vector and Quaternion math libraries.
5.  **Configuration (`robot_spec.h`, `robot_spec.cpp`):** **CRITICAL:** Defines robot dimensions, joint limits, servo mappings, and global state variables.
6.  **Control Interfaces:** Headers implementing logic for specific modes (e.g., `walkcycle_remote.h`, `set_pos.h`).
7.  **Packet Definition (`packets.h`):** Defines the structure of UDP control packets.

## Coordinate Systems Explained

Understanding these frames (as used in the code) is crucial:

1.  **Walk Frame:**
    *   The primary coordinate system for locomotion planning, used internally by `walkcycle.h`.
    *   **Origin & Orientation:** Conceptually, its origin lies on the ground plane below the robot's center. The frame's position and yaw orientation change relative to the external environment based on the commanded `bodyVelocity` and `bodyAngularVelocityYaw`. However, it always remains level (does not pitch or roll relative to the external vertical, Z-Up).
    *   **Command Interpretation:** Commanded `bodyVelocity` and `bodyAngularVelocityYaw` define the instantaneous velocity (linear and angular) that a *stance foot must counteract within the Walk Frame* to remain stationary relative to the external environment.
    *   **Key Use:**
        *   `baseFootPositionWalk[i]` coordinates represent the neutral foot landing targets, fixed *within this Walk Frame's coordinate system*.
        *   The `walkcycle.h` stance phase calculates foot motion *within this frame* needed to achieve external stationarity based on the velocity commands.
        *   The Body Frame's pose (`bodyPositionOffset`, `bodyOrientation`) is defined *relative* to this frame, allowing the physical body to be positioned and oriented independently of the base locomotion defined by the Walk Frame's conceptual movement.

2.  **Body Frame:**
    *   **Origin:** Attached to a reference point on the robot's chassis (e.g., geometric center).
    *   **Orientation:** Fixed relative to the chassis (+Y Forward, +X Right, +Z Up along the chassis).
    *   **Relationship:** Its desired instantaneous position and 3D orientation (yaw, pitch, roll) *relative to the Walk Frame* are specified by `bodyPositionOffset` and `bodyOrientation`. This allows independent control of body pose (tilting, shifting, yawing relative to the direction of travel).

3.  **Leg Frame:**
    *   **Origin:** At the specific leg's coxa joint (hip).
    *   **Orientation:** Rotated relative to the **Body Frame** by the leg's `legMountingAngle` (defined in `robot_spec.cpp`). This aligns the Leg Frame's axes appropriately for the `calculateIK` function (e.g., +Y points horizontally outward along the leg's neutral direction).
    *   **Input Frame:** This is the coordinate system expected by `calculateIK`.

## Kinematic Flow (Remote Walk Mode)

1.  **UDP Commands Received:** The ESP32 receives target values via `FullControlPacket`:
    *   `bodyVelocity`, `bodyAngularVelocityYaw`: Define the instantaneous velocity a stance foot must counteract *within the Walk Frame* to remain externally stationary.
    *   `bodyPositionOffset`, `bodyOrientation`: Define the **Body Frame's** target pose relative to the **Walk Frame**.
    *   `baseFootPositionWalk[i]`: Define neutral foot targets *within the **Walk Frame***.
    *   Gait parameters (`stepHeight`, `stepFrequency`, `dutyFactor`).
    *   Control flags (`walkCycleRunning`).
2.  **Walk Cycle Calculation (`walkcycle.h`):**
    *   Operates entirely *within* the **Walk Frame**.
    *   **Stance Phase:** Calculates the required foot velocity *within the Walk Frame* based *directly* on commanded `bodyVelocity` and `bodyAngularVelocityYaw` to counteract the relative environmental motion, aiming for external stationarity. Updates the foot's **Walk Frame** coordinates (`leg.currentPosition`).
    *   **Swing Phase:** Interpolates the foot from its lift-off position (`swingStartPosition`) to the neutral target (`baseFootPositionWalk[i]`), both defined as coordinates *within the **Walk Frame***. Adds vertical lift (`stepHeight`).
    *   **Output:** Target foot position `P_foot_walk` (in Walk Frame coordinates).
3.  **Body Transformation (`body_transform.h`):**
    *   Takes `P_foot_walk` (Walk Frame coords) as input.
    *   Uses `bodyPositionOffset` and `bodyOrientation` (defining the Body Frame's pose relative to the Walk Frame) to transform the coordinates into the **Body Frame's** reference system.
    *   Uses `legOriginOffset` and `legMountingAngle` (defining the Leg Frame's pose relative to the Body Frame) to transform the coordinates into the **Leg Frame**.
    *   **Output:** Target foot position `P_foot_leg_ik_input` (in Leg Frame coordinates).
4.  **Inverse Kinematics (`ik.h`):**
    *   Takes `P_foot_leg_ik_input` (Leg Frame coords) for a specific leg.
    *   Calculates the required Coxa, Femur, and Tibia joint angles (radians).
5.  **Servo Command (`utils.h`):**
    *   Converts calculated angles to PWM pulse widths based on calibration (`SERVOMIN`/`MAX`) and servo mapping (`LEG_SERVOS`).
    *   Sends commands to the appropriate PCA9685 driver channel.

## Operating Modes (Selectable via Main Menu)

*   **`SERIAL_CONTROL` (`serial_remote.h`):** Low-level servo pulse control via Serial.
*   **`WAVE_PROGRAM` (`servo_wave.h`):** Simple sine wave servo demo.
*   **`IK_PROGRAM` (`set_pos.h`):** Interactive IK testing via Serial (targets relative to Leg Frame).
*   **`WALKCYCLE_SERIAL` (`walkcycle_serial.h`):** Deprecated/Simplified walk control via Serial (may not fully support the current kinematic model).
*   **`WALKCYCLE_REMOTE` (`walkcycle_remote.h`):** **Primary Mode.** Full remote control via UDP using the layered kinematics described above.

## Key Files

*   `hexapod_proj.ino`: Main sketch, state machine.
*   `walkcycle.h`: Core walk cycle logic (operates in Walk Frame).
*   `body_transform.h`: Coordinate transformation (Walk Frame -> Leg Frame).
*   `ik.h`: Inverse kinematics calculation.
*   `robot_spec.h`/`.cpp`: **Robot physical configuration & global state variables.**
*   `Vec3.h` / `quat.h`: Vector and Quaternion math.
*   `utils.h`: PCA9685 driver setup, angle/pulse conversions.
*   `packets.h`: UDP packet structure definition.
*   `walkcycle_remote.h`: UDP handling and remote mode logic.
*   (Other mode headers: `set_pos.h`, `serial_remote.h`, `servo_wave.h`)

## Configuration (CRITICAL!)

You **MUST** configure `robot_spec.cpp` for your specific robot build:

1.  **`LEG_COUNT`:** Set to 6 for a standard hexapod.
2.  **Leg Dimensions:** Define `COXA_LENGTH`, `FEMUR_LENGTH`, `TIBIA_LENGTH` (in cm).
3.  **Joint Limits:** Define `*_MIN_ANGLE`, `*_MAX_ANGLE` for Coxa, Femur, Tibia (in **radians**). Ensure these are respected by IK and potentially PWM conversion.
4.  **`legOriginOffset[LEG_COUNT]`:** Measure the {X, Y, Z} position (cm) of each coxa joint relative to your chosen **Body Frame** origin (+X Right, +Y Fwd, +Z Up along chassis). **Incorrect values here will break body transformations.**
5.  **`legMountingAngle[LEG_COUNT]`:** Determine the physical yaw angle (**radians**, counter-clockwise positive) of each leg's neutral direction relative to the **Body Frame's** +Y axis. Ensure consistency between C++ and any controlling GUI.
6.  **`LEG_SERVOS[LEG_COUNT][3]`:** Map the logical leg/joint (0:Coxa, 1:Femur, 2:Tibia) to the correct physical PCA9685 output channel index (0-17 overall). Double-check against your wiring and the I2C addresses used in `utils.h` (0x40, 0x41).
7.  **Initial State:** Set reasonable initial values for `bodyPositionOffset` (e.g., `{0, 0, 15.0}` for 15cm ride height relative to Walk Frame) and `baseFootPositionWalk` (neutral stance coordinates in **Walk Frame**).
8.  **Edit `walkcycle_remote.h` (if using):**
    *   Set your WiFi `ssid` and `password`.
    *   Adjust `localUdpPort` if needed (must match sender).

## How to Use

1.  **Hardware Setup:** Connect servos, drivers, and ESP32 correctly (check I2C pins in `utils.h`). Provide appropriate power.
2.  **Configure:** **Critically review and edit `robot_spec.cpp` and `walkcycle_remote.h`.**
3.  **Compile & Upload:** Open `hexapod_proj.ino` in Arduino IDE or PlatformIO. Install `Adafruit PWM Servo Driver Library`. Compile and upload.
4.  **Run:** Open Serial Monitor (115200 baud). Select `WALKCYCLE_REMOTE` (or other modes).
5.  **Control (Remote Mode):** Run the companion Python script (`hexapod_controller_gui.py`) or your own UDP sender application to send `FullControlPacket` datagrams to the ESP32's IP address on the configured port. Use the GUI or script to command velocities, pose, and gait parameters.
6.  **Exit Mode:** Usually 'X' via Serial Monitor returns to the main menu.

## Adding New Modes

Follow the pattern in `hexapod_proj.ino`: Add enum, include header, add menu option/handler, add state machine case calling your mode's `update_X()` function.

## Current Limitations & Future Work

*   **Kinematic Model Accuracy:** The sequential application of linear and rotational effects in the Walk Frame stance phase provides good accuracy for typical loop rates (>60Hz) and walking speeds. Extremely high combined speeds/rotations might introduce minor errors.
*   **Gait Flexibility:** Walk cycle uses a simple time-based phase offset (tripod). Developing more sophisticated gaits (ripple, wave) and smooth transitions would be beneficial.
*   **Sensor Integration:** No IMU feedback for balance/stabilization or foot sensors for terrain adaptation.
*   **Error Handling/Safety:** Minimal checks for self-collision, motor limits, etc. Robust safety features should be added.
*   **IK Singularities:** Basic IK doesn't explicitly handle singularities near workspace limits.

## Dependencies

*   Arduino ESP32 Board Support Package
*   `Adafruit PWM Servo Driver Library` (Install via Arduino Library Manager)
*   `WiFi` (ESP32 Built-in)
*   `Wire` (ESP32 Built-in for I2C)