# Quick Start Guide: Running the Hexapod

This guide will help you get the hexapod robot up and running quickly, assuming it has already been assembled and its firmware has been flashed and configured for your target Wi-Fi network.

## Prerequisites

*   The fully assembled Hexapod robot with firmware flashed for your Wi-Fi network.
*   A charged 2S 7.4V LiPo battery (see **CRITICAL BATTERY WARNING** below).
*   A computer with the latest stable version of Python 3 installed (download from [python.org](https://www.python.org/)).
*   Access to the Wi-Fi network the robot is configured for.
*   The robot's project files (cloned from its Git repository).

## !! CRITICAL BATTERY WARNING !!

*   **This robot uses a high-discharge 2S 7.4V LiPo battery. LiPo batteries can be DANGEROUS if misused, over-discharged, or damaged.**
*   **The robot's current onboard voltage reading system is UNRELIABLE/NON-FUNCTIONAL.**
*   **You MUST periodically check the battery voltage MANUALLY using a LiPo battery checker or multimeter. Do NOT let a 2S LiPo fall below ~6.4V under load, or ~7.0V resting, to prevent permanent damage.**
*   Always use a proper LiPo balance charger.
*   Store LiPo batteries safely according to manufacturer guidelines.
*   Handle with care. Disconnect the battery when not in use or when working on the robot.

## Step 1: Clone the Project (If Not Already Done)

1.  Open a terminal or command prompt on your computer.
2.  Clone the project repository:
    ```bash
    git clone <repository-url-of-the-hexapod-project>
    ```
3.  Navigate into the cloned project directory:
    ```bash
    cd <project-directory-name>
    ```

## Step 2: Install Python Dependencies

1.  In your terminal, navigate to the root directory of the cloned project (where `hexapod_gui.py` is located).
2.  It's recommended to use a Python virtual environment for managing dependencies.
    *   Create a virtual environment (e.g., named `.venv`):
        ```bash
        python -m venv .venv
        ```
    *   Activate it:
        *   Windows: `.venv\Scripts\activate`
        *   macOS/Linux: `source .venv/bin/activate`
3.  Install the required Python libraries:
    ```bash
    pip install PySide6 requests
    ```

## Step 3: Power Up the Hexapod

1.  Carefully connect the charged 2S LiPo battery to the hexapod.
2.  Place the robot on a flat, stable surface with enough room for it to move.
3.  Observe the small **yellow status LED** on the XIAO ESP32-S3 Sense board (located on the right side of the USB-C port):
    *   **Attempting Wi-Fi Connection:** The yellow LED will blink.
    *   **Wi-Fi Connection Successful:** The yellow LED will turn solid ON.
    *   **Wi-Fi Connection Failed:** The yellow LED will turn solid OFF.
        *   If the Wi-Fi connection fails (LED is OFF), try fully power cycling the robot. The most effective way is to **disconnect and then reconnect the robot's battery**.
        *   *(Note for developers: If you have the robot connected to your computer via USB for serial monitoring, the USB connection may keep the ESP32 powered even if the battery is removed. In this case, to fully reset the Wi-Fi module, you might need to press the small reset button on the XIAO board or briefly disconnect and reconnect the USB cable as well.)*
4.  If the Wi-Fi connection is successful, and you *optionally* have the robot connected to your computer via USB, the robot's IP address will be printed to the Arduino Serial Monitor (open at 115200 baud). This IP is needed for the GUI if discovery fails.

## Step 4: Run the Python Control GUI

1.  Ensure your computer is connected to the **same Wi-Fi network** as the hexapod.
2.  In your terminal (with the virtual environment activated, if you used one), make sure you are in the root directory of the project.
3.  Run the GUI application:
    ```bash
    python hexapod_gui.py
    ```
    The Hexapod Controller GUI window should appear.

## Step 5: Connect to the Robot via GUI

1.  In the GUI's "Connection" section:
    *   Click the **"Discover Robot"** button. If the robot is on the network and broadcasting its discovery beacon, its IP address and ports should automatically populate the input fields.
    *   **Alternatively**, if discovery fails or you know the robot's IP address (e.g., from the Arduino Serial Monitor), manually enter the robot's IP Address, TCP Port (default: 5006), UDP Port (default: 5005), and MJPEG Port (default: 81).
2.  Click the **"Connect"** button.
3.  The "Status" label should update. Wait for it to show **"Status: Connected & Synced"** in green. This indicates successful communication and state synchronization.

## Step 6: Start Walking!

1.  **IMPORTANT GUI TIP:** Before using keyboard controls, **click on the background of the main GUI window** (or any non-input-field area). This ensures the main window has focus, preventing UI elements from "eating" your key presses.
2.  Once connected and synced, the robot is ready for commands.
3.  **To start/stop walking:** Press the **SPACEBAR** on your keyboard. The "Cmd Walk Active" status in the GUI should update.
4.  **Movement Controls (Keyboard):**
    *   `W`: Move Forward
    *   `S`: Move Backward
    *   `A`: Strafe Left
    *   `D`: Strafe Right
    *   `Q`: Turn Left (Yaw)
    *   `E`: Turn Right (Yaw)
5.  **Body Pose Adjustment Controls (Keyboard):**
    *   Arrow Keys (Up/Down/Left/Right): Adjust body position in X/Y plane.
    *   Shift + Arrow Keys (Up/Down): Adjust body height (Z position).
    *   `I` / `K`: Pitch body Up / Down.
    *   `J` / `L`: Roll body Left / Right.
    *   `U` / `O`: Yaw body Left / Right.
    *   `C` (Hold): Center body X/Y position.
    *   `X` (Hold): Center body orientation (pitch/roll/yaw).
6.  Explore the GUI tabs for adjusting configuration parameters like speed, step height, etc., live.

## Step 7: View the FPV Camera Stream (Optional)

1.  In the GUI, navigate to the "Camera Stream" box (it starts on the "Config" view).
2.  You can typically click **"Start Stream (TCP)"** directly using the default camera settings.
    *   If you wish to change resolution, quality, or FPS limit, make your selections on the "Config" view and click **"Apply Camera Config (TCP)"**. Wait for the "ESP32 ACKed camera config update" message in the GUI's log area at the bottom.
3.  Click the **"Start Stream (TCP)"** button.
4.  If successful, the GUI will switch to the "Stream" view, and the live video feed from the hexapod's camera should appear.
5.  To stop the stream, click **"Stop Stream (TCP)"**.

## Step 8: Powering Down

1.  To stop the robot, press **SPACEBAR** to deactivate the walk cycle.
2.  In the GUI, click **"Disconnect"**.
3.  Carefully disconnect the LiPo battery from the hexapod.
4.  **Remember to check your LiPo battery's voltage MANUALLY!**

## Basic Troubleshooting

*   **Robot Not Found by "Discover Robot":**
    *   Ensure your computer and the hexapod are on the **same Wi-Fi network**.
    *   Check the hexapod's **yellow status LED** to confirm it's connected to Wi-Fi (should be solid ON). If it's OFF, try power cycling the robot by disconnecting and reconnecting its battery.
    *   Try manually entering the robot's IP address if known.
*   **GUI Doesn't Connect (Status remains "Disconnected" or shows an error):**
    *   Double-check the entered IP Address and Port numbers.
    *   Ensure no firewall on your computer is blocking the Python application (`python.exe`) or the specified ports.
    *   Verify the robot is powered on and its yellow status LED indicates a Wi-Fi connection.
*   **Robot Connects ("Synced") but Doesn't Walk:**
    *   **Click the GUI background** to ensure it has focus, then press the **SPACEBAR** to toggle the walk cycle. Check the "Cmd Walk Active" status in the GUI.
    *   **Check the LiPo battery voltage MANUALLY.** A low battery may not provide enough power.
    *   Ensure the robot is on a relatively flat surface suitable for walking.
*   **Keyboard Controls Not Working:**
    *   Make sure the GUI window has focus. Click on a non-interactive part of the GUI (like the background) and try again.
*   **Video Stream Issues:**
    *   Ensure the MJPEG port in the GUI matches the robot's configuration (default 81).
    *   If "Start Stream" fails, check the log area at the bottom of the GUI for error messages from the ESP32.
    *   Try a lower resolution or quality if the stream is very laggy on your network.
*   **Increasing Latency or Connection Problems Over Time:**
    *   Sometimes, network conditions can degrade. Try disconnecting the GUI from the robot and then reconnecting. This can often resolve temporary Wi-Fi related issues.

## Optional: Control with DroidPad (Android)

You can also control the hexapod using the DroidPad app on an Android device. This method offers less overall control than the Python GUI and requires manual IP configuration each time DroidPad starts.

1.  **Install DroidPad:** Download and install DroidPad from the F-Droid app store on your Android device.
2.  **Connect to Wi-Fi:** Ensure your Android device is connected to the **same Wi-Fi network** as the hexapod.
3.  **Configure DroidPad Connection:**
    *   Open DroidPad.
    *   Go to its connection settings (often a Wi-Fi icon or menu option).
    *   Enter the Hexapod's IP Address (you'll need to find this from the Arduino Serial Monitor or the Python GUI after discovery).
    *   Set the Port to **5005** (this is the UDP listening port of the hexapod for control commands).
    *   Attempt to connect in DroidPad.
4.  **Import Control Layout:**
    *   The hexapod project uses a custom DroidPad layout. You can import it by scanning the following QR code from within DroidPad's layout import function (this might be in settings or a layout management area):
        ![DroidPad Layout QR Code](docs/images/droidpad_qr.png)
        *(This QR code image, `droidpad_qr.png`, needs to be present in the `docs/images/` folder of your project repository).*
5.  **Control:** Once DroidPad is connected to the robot's IP and port, and the layout is active, you should be able to take control of the robot by flipping the toggle switch. The DroidPad interface will send joystick and button data to the hexapod, which interprets it similarly to the keyboard inputs.

---