# ESP32-S3 Hexapod Control Software

This repository contains the C++ firmware for controlling an 18-DOF hexapod robot based on an ESP32-S3 microcontroller. It features a state machine architecture allowing various control modes, including inverse kinematics (IK) testing, walk cycle control via serial, and remote walk cycle control via UDP/WiFi. The kinematic system uses a layered approach with coordinate transformations to enable independent body pose control alongside locomotion.

## Project Goal

The primary goal is to develop a versatile and extensible control system for a hexapod robot, enabling remote control of walking gaits with independent control over the robot body's position and orientation relative to the ground.

## Hardware

*   **Microcontroller:** Xiao ESP32-S3
*   **Servo Motors:** 18x High-Torque Hobby Servos (e.g., 35kg-cm), 3 per leg (Coxa, Femur, Tibia).
*   **Servo Driver:** 2x Adafruit PCA9685 16-Channel PWM Servo Driver boards (connected via I2C). Each board drives 9 servos (3 legs).
*   **Connectivity:** WiFi (for UDP remote control).
*   **(Optional):** Remote control transmitter/receiver or PC for sending UDP commands.

## Software Architecture

The software is built around several key components:

1.  **State Machine (`hexapod_proj.ino`):** Manages the overall program flow, allowing the user to select different operating modes via the Serial Monitor. Each mode has dedicated `setup_X()` and `update_X()` functions.
2.  **Kinematic Layers:**
    *   **Walk Cycle (`walkcycle.h`):** Generates desired foot target positions (`P_foot_walk`) in a stable, world-aligned "Walk Frame" based on step parameters and desired linear `bodyVelocity`.
    *   **Body Transformation (`body_transform.h`):** Transforms Walk Frame targets into the local frame required by the IK solver for each leg. It accounts for the desired `bodyPositionOffset` and `bodyOrientation` (relative to the Walk Frame) and each leg's physical `legOriginOffset` and `legMountingAngle` on the chassis.
    *   **Inverse Kinematics (`ik.h`):** Calculates the required Coxa, Femur, and Tibia joint angles (`calculateIK`) to reach the transformed target position for a specific leg.
3.  **Low-Level Control (`utils.h`):** Handles setup and communication with the PCA9685 servo drivers via I2C, converting desired angles (degrees or radians) into PWM pulse widths.
4.  **Math Utilities:**
    *   `Vec3.h`: Vector math operations.
    *   `quat.h`: Quaternion math for representing and manipulating orientations.
5.  **Configuration (`robot_spec.h`, `robot_spec.cpp`):** Centralizes all robot-specific physical parameters (leg lengths, offsets, mounting angles, joint limits, servo pins) and global state variables (desired body pose).
6.  **Control Interfaces:** Header files implementing the logic and serial/UDP communication for each state machine mode (e.g., `walkcycle_serial.h`, `walkcycle_remote.h`, `set_pos.h`).

## Operating Modes (Selectable via Main Menu)

The state machine (`hexapod_proj.ino`) provides the following modes:

1.  **`SERIAL_CONTROL` (`serial_remote.h`):** Direct, low-level control of individual servo pulse widths via Serial commands. Useful for basic hardware checks and servo calibration.
2.  **`WAVE_PROGRAM` (`servo_wave.h`):** Simple demonstration program cycling servos through a sine wave pattern.
3.  **`IK_PROGRAM` (`set_pos.h`):** Interactive Inverse Kinematics testing via Serial Monitor. Allows selecting individual legs or all legs, setting target X, Y, Z coordinates (relative to the leg's origin), and commanding movement. Good for verifying IK calculations and reachability.
4.  **`WALKCYCLE_SERIAL` (`walkcycle_serial.h`):** Controls the walk cycle (`walkcycle.h`) using Serial commands. Allows setting linear `bodyVelocity`, step height, and step frequency to test walking gaits and parameters locally.
5.  **`WALKCYCLE_REMOTE` (`walkcycle_remote.h`):** (Primary Goal) Controls the walk cycle using commands received over WiFi via UDP packets (sent from a companion script like `hexapod_remote_sender.py` or another application). This mode enables untethered remote control. *(Note: UDP packet protocol is currently under development and may evolve).*

## Coordinate Systems

*   **Body Frame:** Origin at robot center; +X Right, +Y Forward, +Z Up (relative to chassis). `legOriginOffset` defined here.
*   **Walk Frame:** Origin on ground below body; +X Global Right, +Y Global Forward, +Z Global Up (gravity aligned). `walk_cycle.h` outputs targets (`P_foot_walk`) in this frame. `bodyPositionOffset` & `bodyOrientation` are relative to this frame.
*   **Leg Local IK Frame:** Origin at leg's coxa joint; +Y axis points along the leg's neutral forward direction (based on `legMountingAngle`), +X points right relative to the leg, +Z points up. This is the frame expected by `calculateIK`.

## Key Files

*   `hexapod_proj.ino`: Main sketch, state machine logic, `setup()`, `loop()`.
*   `walkcycle.h`: Core walk cycle algorithm (time-based, linear velocity only currently).
*   `ik.h`: Inverse kinematics calculations (`calculateIK`).
*   `body_transform.h`: `transformWalkFrameToLegFrame` function (Walk Frame -> Leg IK Frame).
*   `robot_spec.h`/`.cpp`: **Robot configuration (dimensions, angles, pins, limits) & global state.**
*   `Vec3.h` / `quat.h`: Vector and Quaternion math.
*   `utils.h`: PWM driver setup, angle/pulse conversions, helper functions.
*   `walkcycle_serial.h`, `walkcycle_remote.h`, `set_pos.h`, `serial_remote.h`, `servo_wave.h`: Implementations for each specific operating mode.

## Configuration

**Crucially, you MUST configure the software for your specific robot build:**

1.  **Edit `robot_spec.h`:** Review the `extern` declarations.
2.  **Edit `robot_spec.cpp`:**
    *   Set `LEG_COUNT` (should be 6 for a hexapod).
    *   Define `COXA_LENGTH`, `FEMUR_LENGTH`, `TIBIA_LENGTH`.
    *   Define `*_MIN_ANGLE`, `*_MAX_ANGLE` joint limits (in radians).
    *   Define `legOriginOffset[LEG_COUNT]`: Measure the {X, Y, Z} position of each coxa joint relative to your chosen body center (using Body Frame axes).
    *   Define `legMountingAngle[LEG_COUNT]`: Determine the physical angle (in radians, counter-clockwise from Body +Y is recommended) of each leg's neutral direction.
    *   Define `LEG_SERVOS[LEG_COUNT][3]`: Map the correct PCA9685 output channel (0-15 range, considering which board handles which leg) to each joint (0:Coxa, 1:Femur, 2:Tibia) for each leg. Pay attention to the driver chip addressing (0x40, 0x41) and channel mapping in `utils.h`.
    *   Set initial `bodyPositionOffset` (e.g., `{0, 0, 15.0}` for a 15cm ride height).
3.  **Edit `walkcycle_remote.h` (if using):**
    *   Set your `ssid` and `password` for WiFi connection.
    *   Adjust `localUdpPort` if needed.

## How to Use

1.  **Hardware Setup:** Connect servos to PCA9685 boards, connect boards to ESP32-S3 via I2C (check pins in `utils.h`), power servos appropriately.
2.  **Configure:** Edit `robot_spec.cpp` and potentially `walkcycle_remote.h` as described above.
3.  **Compile & Upload:** Open `hexapod_proj.ino` in the Arduino IDE (with ESP32 board support installed) or PlatformIO. Install the `Adafruit PWM Servo Driver Library`. Compile and upload to the Xiao ESP32-S3.
4.  **Run:** Open the Arduino Serial Monitor (set baud rate to 115200). You will see the Main Menu.
5.  **Select Mode:** Enter the number corresponding to the desired operating mode.
6.  **Interact:** Follow the specific command prompts displayed for the chosen mode (e.g., use 'x', 'y', 'z' commands in IK mode, 'G'/'B' in walk cycle modes, send UDP packets in remote mode).
7.  **Exit Mode:** Most modes use 'X' to return to the Main Menu.

## Adding New Modes

The state machine makes adding new test modes or control methods straightforward:

1.  **Create Header:** Create a new `.h` file (e.g., `my_new_mode.h`).
2.  **Implement Functions:** Define `setup_my_new_mode()` and `bool update_my_new_mode()` within the new header. The update function should return `false` to signal an exit back to the main menu, `true` otherwise.
3.  **Add Enum:** Add a new state identifier (e.g., `MY_NEW_MODE`) to the `ProgramState` enum in `hexapod_proj.ino`.
4.  **Include Header:** Include your new header file in `hexapod_proj.ino`.
5.  **Add Menu Option:** Add a corresponding option in `printMainMenu()` and `handleMainMenu()` in the `.ino` file.
6.  **Add State Case:** Add a `case MY_NEW_MODE:` block within `stateMachine()` in the `.ino` file, calling your `update_my_new_mode()` function.

## Current Limitations & Future Work

*   **Walk Cycle Turning:** Walk cycle planning (`walkcycle.h`) does not account for desired angular velocity. Turning while moving forward relies solely on the reactive body transformation and will likely cause foot scuffing. -> *Integrate angular velocity (body yaw rate) into `targetTouchdownPos` calculation.*
*   **UDP Protocol:** The `WalkControlPacket` in `walkcycle_remote.h` is basic. -> *Expand protocol to include desired `bodyPositionOffset`, `bodyOrientation`, and potentially angular velocity.*
*   **Body Pose Control Input:** No explicit interface currently exists in walk cycle modes to dynamically command `bodyPositionOffset` or `bodyOrientation`. These are currently set by global variables, potentially modified by UDP packets in the future.
*   **Gait Flexibility:** Walk cycle implements a simple time-based phase offset. -> *Develop a more sophisticated gait controller (e.g., tripod, ripple, wave) with smooth transitions.*
*   **Sensor Integration:** No use of IMU for balance/stabilization or foot contact sensors for terrain adaptation.
*   **Error Handling/Safety:** Minimal safety checks (e.g., for self-collision, motor overheating, excessive joint torques).
*   **IK Singularities:** Basic IK does not specifically handle kinematic singularities.

## Dependencies

*   Arduino ESP32 Board Support Package
*   `Adafruit PWM Servo Driver Library` (Install via Arduino Library Manager)
*   `WiFi` (Built-in for ESP32)
*   `Wire` (Built-in for I2C)