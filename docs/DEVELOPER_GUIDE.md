# Developer's Guide

This guide is intended for those who want to understand the Hexapod's software and firmware in more detail, set up the development environment, make modifications, or contribute to the project.

## Table of Contents
1.  [Setting up the ESP32 Firmware Development Environment](#1-setting-up-the-esp32-firmware-development-environment)
2.  [Setting up the Python GUI Development Environment](#2-setting-up-the-python-gui-development-environment)
3.  [Project File Structure Overview](#3-project-file-structure-overview)
4.  [Understanding the Code (High-Level)](#4-understanding-the-code-high-level)
    *   [Firmware (ESP32)](#firmware-esp32)
    *   [Python GUI & Comms](#python-gui--comms)
5.  [Key Configuration Files/Sections](#5-key-configuration-filessections)
6.  [Modifying and Extending the Project](#6-modifying-and-extending-the-project)
7.  [Troubleshooting Development Issues](#7-troubleshooting-development-issues)

---

## 1. Setting up the ESP32 Firmware Development Environment

The Hexapod's firmware is developed using the Arduino IDE with the ESP32 board support package.

### 1.1. Install Arduino IDE
*   Download and install the latest version of the Arduino IDE from [www.arduino.cc/en/software](https://www.arduino.cc/en/software).

### 1.2. Select the right board
1.  Open the Arduino IDE.
2.  Go to `File > Preferences` (or `Arduino IDE > Settings...` on macOS).
3.  In the "Additional Board Manager URLs" field, ensure the following URL is present (add it if it's missing):
    ```
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    ```
4.  Click "OK".
5.  Go to `Tools > Board > Boards Manager...`.
6.  Search for "esp32". If a package by "Espressif Systems" is listed, ensure it's installed and up to date. If not, install it. This package provides support for a wide range of ESP32-based boards.

### 1.3. Select the Board and Port
1.  Once the ESP32 boards package is installed, go to `Tools > Board > ESP32 Arduino` and select **"XIAO_ESP32S3"** from the list.
2.  Connect the XIAO ESP32-S3 Sense to your computer via a USB-C cable.

### 1.3. Select the Board
*   After installation, go to `Tools > Board > ESP32 Arduino` and select **"XIAO_ESP32S3"**.
    *   Ensure other board settings (like Upload Speed, Partition Scheme, etc.) are at their defaults unless you have a specific reason to change them. The default " énorme APP (3MB No OTA/1MB SPIFFS)" should be fine.

### 1.4. Install Necessary Arduino Libraries
You will need to install the following libraries using the Arduino Library Manager (`Sketch > Include Library > Manage Libraries...`):
1.  **Adafruit PWM Servo Driver Library**
    *   Search for "Adafruit PWM Servo Driver" and install the library by Adafruit.
2.  **ArduinoJson**
    *   Search for "ArduinoJson" and install the library by **Benoit Blanchon**.
    *   **Important:** Do *not* install the library named "Arduino_JSON" by Arduino, as it is a different library and will not work with this project.

### 1.5. Configure Wi-Fi Credentials
1.  Open the `passwords.h` file from the project.
2.  Modify the `SSID` and `PASSWORD` definitions to match your Wi-Fi network:
    ```cpp
    #define SSID "YourWifiNetworkName"
    #define PASSWORD "YourWifiPassword"
    ```
    *(Note: You can use preprocessor directives like `#ifdef GUESTNET` to manage multiple Wi-Fi configurations if needed, as seen in the example code).*

### 1.6. Compiling and Uploading Firmware
1.  With the correct board and port selected, click the "Upload" button (the right-arrow icon) in the Arduino IDE to compile and flash the firmware to the ESP32.
2.  After uploading, open the Serial Monitor (`Tools > Serial Monitor` or `Shift+Ctrl+M` / `Shift+Cmd+M`).
3.  Set the baud rate in the Serial Monitor to **9600** to view log messages from the robot, including its IP address upon successful Wi-Fi connection. *(The default should be 9600 baud already).*

---

## 2. Setting up the Python GUI Development Environment

The control GUI is a Python application using the PySide6 library.

### 2.1. Install Python
*   Ensure you have Python 3 installed (latest stable version recommended). Download from [python.org](https://www.python.org/).

### 2.2. Create a Virtual Environment (Recommended)
1.  Open a terminal or command prompt.
2.  Navigate to the root directory of the hexapod project.
3.  Create a virtual environment:
    ```bash
    python -m venv .venv
    ```
4.  Activate it:
    *   Windows: `.venv\Scripts\activate`
    *   macOS/Linux: `source .venv/bin/activate`

### 2.3. Install Python Dependencies
*   With the virtual environment activated, install the necessary libraries:
    ```bash
    pip install PySide6 requests
    ```

---

## 3. Project File Structure Overview

The project has a flat structure for firmware files and a few Python scripts at the root for the GUI and utilities.

*   **`hexapod_proj.ino`**: The main Arduino sketch. Contains `setup()` and `loop()` functions, orchestrates different operating modes, and handles basic serial menu commands.
*   **`passwords.h`**: Stores Wi-Fi credentials (SSID and Password). **Crucial to edit for your network.**
*   **`robot_spec.h` / `robot_spec.cpp`**: Defines fundamental robot parameters:
    *   Leg segment lengths (`COXA_LENGTH`, `FEMUR_LENGTH`, `TIBIA_LENGTH`).
    *   Joint angle limits.
    *   Servo channel mappings (`LEG_SERVOS`).
    *   Default leg mounting angles (`legMountingAngle`).
    *   Default coxa joint offsets from the body center (`legOriginOffset`).
    *   Default base foot positions for the walk cycle (`baseFootPositionWalk`).
    *   Global state variables for body pose and velocity.
*   **`servo_angles.h` / `servo_angles.cpp`**:
    *   Manages PCA9685 servo driver initialization (`setupPwm`).
    *   Functions for converting logical angles (degrees/radians) to PWM pulse values (`get_pulse_from_angle_degrees`, `setAngleRadians`, etc.).
    *   Defines servo pulse limits (`SERVOMIN`, `SERVOMAX`) and PWM frequency.
*   **`math_utils.h` / `math_utils.cpp`**: Provides `Vec3` and `Quaternion` classes, along with common math operations (dot/cross products, normalization, slerp, mapf, clampf).
*   **`ik.h`**: Contains the `calculateIK` function for performing inverse kinematics for a single leg.
*   **`body_transform.h`**: Includes `transformWalkFrameToLegFrame` for converting desired foot positions from the walk frame to the leg's local IK input frame, considering body pose.
*   **`walkcycle.h` / `walkcycle.cpp`**: Implements the tripod gait logic, step generation, and leg phase management. Uses `body_transform.h` and `ik.h` to command leg movements.
*   **`remote_control.h` / `remote_control.cpp`**:
    *   Handles all remote control operations.
    *   Initializes and manages network communication via `network_comms`.
    *   Parses incoming JSON commands (from Python GUI or DroidPad).
    *   Updates robot state variables based on commands (e.g., target velocities, pose adjustments).
    *   Manages telemetry subscriptions and sends data back to the GUI.
    *   Integrates DroidPad inputs.
    *   Manages the discovery beacon.
*   **`network_comms.h` / `network_comms.cpp`**: Low-level TCP and UDP server/client functionalities for the ESP32. Handles packet reception, parsing, and sending.
*   **`camera_pins.h`**: Defines the GPIO pin assignments for the XIAO ESP32-S3 Sense camera module. **Critical if using a different ESP32 board or camera wiring.**
*   **`streamer.h` / `streamer.cpp`**: Initializes the camera and runs an HTTP server to stream MJPEG video. Handles camera parameter settings (resolution, quality, FPS).
*   **`servo_test_mode.h`**: Implements a serial-driven mode for testing individual servos.
*   **`battery.h`**: Contains code for reading battery voltage (currently marked as unreliable).
*   ---
*   **`hexapod_gui.py`**: The main Python application for the control GUI, built with PySide6.
*   **`hexapod_comms_client.py`**: Python class used by the GUI to manage TCP/UDP communication with the ESP32.
*   **`simple_mjpeg_viewer.py`**: A standalone Python script for viewing MJPEG streams (can be used for testing the camera stream independently of the main GUI).
*   **`tcp_sniffer.py`**, **`udp_logger.py`**: Python utility scripts for debugging network communication.
*   ---
*   **`/docs/`**: Contains all documentation files, including this one.
    *   **`/docs/images/`**: Stores images used in the documentation.

---

## 4. Understanding the Code (High-Level)

### Firmware (ESP32)
The ESP32 firmware is responsible for all real-time control and hardware interaction.
*   **Startup (`hexapod_proj.ino` -> `setup()`):** Initializes serial, PWM drivers, Wi-Fi, and then sets up the `remote_control` and `servo_test_mode` modules.
*   **Main Loop (`hexapod_proj.ino` -> `loop()`):** A state machine that calls the update function for the `currentOperatingMode` (e.g., `remoteControlUpdate()`).
*   **Remote Control Mode (`remote_control.cpp`):**
    *   Listens for incoming TCP/UDP JSON packets using `network_comms.cpp`.
    *   `process_json_packet_internal` routes packets based on their "type" field to specific handlers (e.g., `processLocomotionIntent`, `processConfigUpdate`).
    *   Updates global state variables in `robot_spec.h` (like `bodyVelocity`, `bodyPositionOffset`, `walkParams`).
    *   Calls `updateWalkCycle()` if `walkCycleRunning` is true.
    *   Sends telemetry data back to the connected GUI based on active subscriptions.
    *   Manages the camera stream start/stop via `streamer.cpp`.
*   **Walk Cycle & Kinematics:**
    *   `walkcycle.cpp` calculates desired foot target positions in the "Walk Frame" based on the current gait phase and desired robot motion.
    *   `body_transform.h` converts these Walk Frame targets into coordinates suitable for each leg's individual Inverse Kinematics (IK) solver, taking into account the robot's current body position and orientation.
    *   `ik.h` performs the IK calculation for each leg to find the required coxa, femur, and tibia joint angles.
    *   `servo_angles.cpp` converts these angles into PWM signals and sends them to the PCA9685 servo drivers.
*   **Networking (`network_comms.cpp`):** Provides a generalized layer for receiving JSON packets over TCP and UDP, and sending JSON responses/telemetry.
*   **Camera Streaming (`streamer.cpp`):** Configures the ESP32 camera and runs a simple HTTP server that streams JPEG frames as an MJPEG video.

### Python GUI & Comms
*   **`hexapod_gui.py`:**
    *   Uses PySide6 for the user interface.
    *   Instantiates `HexapodCommsClient` to communicate with the robot.
    *   Sends user inputs (keyboard, button clicks) as JSON commands to the ESP32.
    *   Receives and displays telemetry and state information from the ESP32.
    *   Connects to the MJPEG stream from the ESP32 and displays it.
*   **`hexapod_comms_client.py`:**
    *   Manages the TCP connection (connect, disconnect, send, receive).
    *   Handles sending UDP packets.
    *   Parses incoming JSON messages from the ESP32 (TCP).
    *   Emits Qt signals when messages are received or connection status changes.

---

## 5. Key Configuration Files/Sections

When modifying or adapting the robot, these are the primary areas you'll likely need to adjust:

*   **`passwords.h`**: **Wi-Fi Credentials (SSID & Password).** Must be changed for your network.
*   **`robot_spec.h` / `robot_spec.cpp`**:
    *   `COXA_LENGTH`, `FEMUR_LENGTH`, `TIBIA_LENGTH`: Physical lengths of leg segments. **Critical if you change the robot's physical structure.**
    *   `COXA_MIN_ANGLE` to `TIBIA_MAX_ANGLE`: Software limits for joint angles.
    *   `LEG_SERVOS[][]`: Mapping of logical leg/joint to physical PCA9685 servo channel. **Crucial if your servo wiring changes.**
    *   `legOriginOffset[]`: Position of each leg's coxa joint relative to the body center.
    *   `legMountingAngle[]`: Mounting angle (yaw) of each leg.
    *   `servo_center_angle[]`: Fine-tuning offsets if your servos' mechanical neutral isn't perfectly aligned with their electrical center when commanded to logical 0 degrees.
    *   `baseFootPositionWalk[]`: Default neutral stance positions for each foot in the Walk Frame. Defines the center of the stepping envelope.
*   **`camera_pins.h`**: GPIO pin definitions for the XIAO ESP32-S3 Sense camera. **Only change if you are re-wiring the camera or using a different ESP32 board.**
*   **`servo_angles.h`**:
    *   `SERVOMIN`, `SERVOMAX`: Pulse width values corresponding to the minimum and maximum angles your servos can safely reach (typically around -90 to +90 degrees from their center). **Important for servo calibration to avoid damage.**
    *   `SERVO_FREQ`: PWM frequency for the servos (usually 50Hz for analog, can be higher for some digital servos).
*   **`remote_control.h` (Constants):**
    *   `UDP_LISTEN_PORT`, `TCP_LISTEN_PORT`: Network ports the ESP32 listens on. Must match GUI configuration.
    *   `DISCOVERY_BROADCAST_PORT`: Port used for network discovery beacons.
*   **`streamer.cpp` (Constants):**
    *   `PART_BOUNDARY`: String used in MJPEG streaming. Should not need changing unless debugging MJPEG clients.
    *   Default camera settings (initial framesize, quality) are set in `streamer_init_camera`.
*   **Python GUI (`hexapod_gui.py` - top constants):**
    *   `DEFAULT_ROBOT_TARGET_IP`, `DEFAULT_ROBOT_TCP_PORT`, etc.: Default connection parameters for the GUI.

---

      
## 6. Modifying and Extending the Project

### Adding New Commands (Firmware & GUI)
1.  **Define JSON Structure:** Decide on the "type" string and "payload" structure for your new command.
2.  **Firmware (ESP32 - `remote_control.cpp`):**
    *   Add a new `else if` case in `process_json_packet_internal` to handle your new "type".
    *   Create a new processing function (e.g., `processMyNewCommand(JsonObjectConst payload)`).
    *   Implement the logic in this new function to act on the payload and update robot state or call other functions.
3.  **GUI (Python - `hexapod_comms_client.py`):**
    *   Add a new method to `HexapodCommsClient` to send your new command (e.g., `send_my_new_command(self, param1, param2)`). This method will construct the JSON dictionary and use `_send_tcp()` or `_send_udp()`.
4.  **GUI (Python - `hexapod_gui.py`):**
    *   Add UI elements (buttons, sliders, etc.) to trigger your new command.
    *   In the UI element's event handler (e.g., button click slot), call the new method you added to `self.comms_client`.

### Adding a New Operating Mode (Firmware State Machine)
The main operating mode is controlled by the `currentOperatingMode` variable and a `switch` statement in `hexapod_proj.ino`. To add a new mode:
1.  **Define Enum:** Add your new mode to the `RobotOperatingMode` enum in `hexapod_proj.ino`:
    ```cpp
    enum RobotOperatingMode {
      MAIN_MENU,
      REMOTE_CONTROL,
      SERVO_TEST,
      MY_NEW_MODE // Add your new mode here
    };
    ```
2.  **Create Module Files:** Create new `.h` and `.cpp` files for your mode (e.g., `my_new_mode.h`, `my_new_mode.cpp`).
    *   In `my_new_mode.h`, declare `setupMyNewMode()` and `bool myNewModeUpdate()` functions.
    *   In `my_new_mode.cpp`, implement these functions. `myNewModeUpdate()` should return `false` if it wants to exit back to the `MAIN_MENU`.
3.  **Include and Setup:**
    *   `#include "my_new_mode.h"` in `hexapod_proj.ino`.
    *   Call `setupMyNewMode();` within the main `setup()` function in `hexapod_proj.ino`.
4.  **Add to Main Menu:**
    *   In `printMainMenuHelp()` in `hexapod_proj.ino`, add a serial command character and description for entering your new mode.
    *   In `processMainMenuCommands()` in `hexapod_proj.ino`, add a `case` for your new command character that sets `currentOperatingMode = MY_NEW_MODE;` and perhaps calls a function to print help for your new mode.
5.  **Add to Main Loop:**
    *   In the main `loop()` function's `switch(currentOperatingMode)` block in `hexapod_proj.ino`, add a `case MY_NEW_MODE:`:
      ```cpp
      case MY_NEW_MODE:
        if (!myNewModeUpdate()) {
          Serial.println("Exiting My New Mode, returning to Main Menu.");
          currentOperatingMode = MAIN_MENU;
          printMainMenuHelp();
        }
        break;
      ```
6.  **Exiting a Mode:** To exit any mode (e.g., `REMOTE_CONTROL`, `SERVO_TEST`, or your new mode) and return to the main menu, the respective `...Update()` function should return `false`. The main loop in `hexapod_proj.ino` handles transitioning back to `MAIN_MENU` and printing its help. The `REMOTE_CONTROL` mode, for instance, can be exited by receiving an 'X' command over serial while it's active (this logic is within `remoteControlUpdate()`).

### AI for Code Understanding & Modification
This project was developed with significant AI assistance (Gemini 2.5 Pro on aistudio.google.com), particularly for parts of the Python GUI. If you are looking to understand specific sections of code or plan modifications:
*   **Utilize a capable AI model with a large context window:** The key to effectively using AI for this project has been providing it with as much relevant context as possible. Instead of feeding isolated snippets, **uploading multiple related files or even the entire relevant codebase** (e.g., all firmware files if working on the ESP32, or all Python GUI files if working on the client) to an AI like Gemini 2.5 Pro on aistudio.google.com can yield much better results.
*   **Ask targeted questions:**
    *   "Given these files (`file1.cpp`, `file2.h`, `main.ino`), explain how the `XYZ_FEATURE` is implemented."
    *   "How does the `HexapodCommsClient` class in `hexapod_comms_client.py` handle incoming TCP messages from the ESP32 described in `network_comms.cpp`?"
    *   "Based on the overall project structure provided, suggest how I might add [new feature] that involves changes in both the firmware and the Python GUI."
*   **Iterate and provide feedback:** AI development is rapid, and the best practices for prompting and providing context may evolve. Experiment with how you present the code and your questions. What works best today might be different in a few months. The core idea remains: more relevant context generally leads to more accurate and helpful AI assistance.

---

## 7. Troubleshooting Development Issues

### Firmware Compilation/Upload Problems
*   **Library Not Found:** Ensure all required Arduino libraries (see section 1.4) are correctly installed via the Library Manager. Double-check names (e.g., `ArduinoJson` by Benoit Blanchon, not `Arduino_JSON`).
*   **Board Not Selected/Incorrect Port:** Verify that "XIAO_ESP32S3" is selected under `Tools > Board` and the correct COM port is chosen under `Tools > Port`.
*   **Upload Errors:**
    *   Try holding down the "BOOT" button on the XIAO board while clicking "Upload" in the IDE, then release "BOOT" when the IDE shows "Connecting...".
    *   Ensure you have a good quality USB-C data cable.
    *   Try a different USB port on your computer.

### Wi-Fi Connection Issues (During Development)
*   **Check `passwords.h`:** Ensure the SSID and password are correct for the target Wi-Fi network.
*   **Serial Monitor:** Observe the output in the Arduino Serial Monitor (115200 baud) for error messages or connection status.
*   **Power Cycling for Wi-Fi Reset:**
    *   **If USB is connected (for serial monitoring/power):** The ESP32 and its Wi-Fi module may remain powered by USB even if the main robot battery is disconnected. To perform a full reset of the Wi-Fi system in this state:
        1.  Press the small **reset button** on the XIAO ESP32-S3 board.
        2.  Alternatively, briefly **disconnect and reconnect the USB cable** from the computer or the XIAO board. This ensures the ESP32 fully re-initializes.
    *   **If only battery powered:** Disconnecting and reconnecting the main LiPo battery is sufficient to power cycle the entire robot, including the ESP32 and its Wi-Fi.
*   **Network Issues:** Ensure the Wi-Fi network is functioning correctly and is a 2.4GHz network (ESP32s typically do not support 5GHz Wi-Fi). Check for MAC filtering or other restrictive settings on the Wi-Fi router.

### Python GUI Issues
*   **Dependencies Not Met:** Ensure `PySide6` and `requests` are installed in your active Python environment (preferably a virtual environment).
*   **Connection Errors (GUI Log):** Check the terminal log area at the bottom of the GUI for detailed error messages regarding TCP/UDP connection failures.
    *   "Connection Refused": Robot's IP/port is incorrect, or the robot's TCP/UDP server isn't running/listening.
    *   "Timeout": Robot is not responding, possibly a network issue or the robot is unresponsive.
*   **Keyboard Controls Not Working in GUI:** Make sure the main GUI window has focus by clicking on its background before using keyboard shortcuts.

### Kinematics/Walking Problems
*   **Incorrect `robot_spec.h` values:** Double-check all leg lengths, servo mappings, mounting angles, and origin offsets. These are fundamental to correct movement.
*   **Servo Calibration (`servo_angles.h`):** Ensure `SERVOMIN`, `SERVOMAX`, and `servo_center_angle[]` are correctly calibrated for your specific servos. Incorrect values can lead to limited movement, jittering, or servo damage. Use the `servo_test_mode.h` to help calibrate.
*   **IK Math:** If making deep changes to IK or transformations, verify the math carefully. The "Technical Deep Dive: Kinematics & Walk Cycle" document can be a reference.