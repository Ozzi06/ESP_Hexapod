# Technical Deep Dive: Communication Protocol

This document provides an overview of the communication methods and data flow between the Python control GUI and the ESP32-S3 hexapod firmware. The system uses TCP for reliable control, UDP for low-latency intents, and HTTP for video streaming.

## 1. Communication Channels & Purpose

The hexapod system relies on three primary network protocols:

*   **TCP (Transmission Control Protocol):**
    *   **Use:** Establishes a reliable, ordered connection between the GUI and the ESP32 (ESP32 acts as server on default port 5006).
    *   **Data Flow (GUI -> ESP32):**
        *   Connection initiation.
        *   Sending comprehensive configuration updates (e.g., movement speeds, gait parameters, leg geometry).
        *   Sending critical commands requiring acknowledgement (e.g., start/stop walk cycle, camera control commands, system reset commands).
        *   Client settings, including telemetry subscription preferences.
        *   Ping requests for connection health and RTT.
        *   Graceful disconnect notices.
    *   **Data Flow (ESP32 -> GUI):**
        *   Acknowledgements for received commands.
        *   Full robot state and configuration responses (when requested).
        *   Reliable, less frequent telemetry (e.g., battery voltage, Wi-Fi RSSI) based on GUI subscriptions.
        *   Pong replies.
    *   **Managed by:** `hexapod_comms_client.py` (GUI) and `network_comms.cpp` (ESP32).

*   **UDP (User Datagram Protocol):**
    *   **Use:** Facilitates fast, low-latency communication for data where occasional packet loss is acceptable.
    *   **Data Flow (GUI -> ESP32):**
        *   ESP32 listens for commands on default port 5005.
        *   Frequent transmission of locomotion intents (desired forward/strafe/turn speeds derived from keyboard/joystick/DroidPad).
        *   Frequent transmission of body pose adjustment intents (e.g., requests to shift body, tilt, or apply centering).
    *   **Data Flow (ESP32 -> GUI):**
        *   GUI listens for telemetry on a separate port (default 5007).
        *   Frequent transmission of actual robot state (e.g., current velocities, body pose, gait status) if subscribed by the GUI.
        *   Optional debug information like per-leg foot positions.
    *   **Managed by:** `hexapod_comms_client.py` (GUI sending), `TelemetryReceiverUDP` in `hexapod_gui.py` (GUI receiving), and `network_comms.cpp` (ESP32).

*   **UDP Broadcast (for Discovery):**
    *   **Use:** Allows the ESP32 to announce its presence on the network so the GUI can easily find it.
    *   **Data Flow (ESP32 -> Network):**
        *   ESP32 periodically broadcasts a "discovery beacon" packet (containing its IP and service ports) to a specific port (default 5008).
    *   **Data Flow (Network -> GUI):**
        *   The GUI listens on the discovery port (via `DiscoveryReceiverUDP` in `hexapod_gui.py`) to detect these beacons.
    *   **Managed by:** `remote_control.cpp` (ESP32 broadcasting) and `hexapod_gui.py` (GUI listening).

*   **HTTP (Hypertext Transfer Protocol) for MJPEG Stream:**
    *   **Use:** Streams live video from the hexapod's camera.
    *   **Data Flow (ESP32 -> GUI):**
        *   The ESP32 (`streamer.cpp`) runs a lightweight HTTP server on a specific port (default 81).
        *   This server provides an MJPEG (Motion JPEG) video stream at a `/stream` endpoint.
        *   The Python GUI (`MjpegStreamWorker` in `hexapod_gui.py`) acts as an HTTP client, connecting to this endpoint to receive and display the sequence of JPEG frames.
    *   **Managed by:** `streamer.cpp` (ESP32 server) and `hexapod_gui.py` (GUI client).

## 2. Data Format: JSON

*   All control commands, telemetry data, and configuration messages exchanged over TCP and UDP are formatted as **JSON (JavaScript Object Notation) strings.**
*   For TCP, messages are typically newline-terminated (`\n`) to help the receiver delineate individual JSON objects within the stream.
*   A general structure often followed is:
    ```json
    {
      "type": "message_type_string", // e.g., "config_update", "locomotion_intent"
      "source": "sender_id_string",  // e.g., "python_gui", "esp32_hexapod"
      "payload": { /* message-specific data */ }
    }
    ```
    The `"type"` field is crucial for determining how a message should be processed.

## 3. Communication Workflow Example (GUI Connecting & Sending Commands)

1.  **Discovery (Optional):** GUI listens for UDP broadcast beacons from the ESP32. If a beacon is received, the GUI can auto-populate the ESP32's IP and port details.
2.  **TCP Connection:** GUI initiates a TCP connection to the ESP32's IP address and TCP command port (e.g., 5006).
3.  **Client Settings & State Sync:**
    *   Once TCP is connected, the GUI sends a `client_settings` message to the ESP32. This message informs the ESP32 where to send UDP telemetry (GUI's IP and listening port) and which telemetry topics the GUI subscribes to (both TCP and UDP).
    *   The GUI then typically sends a `request_full_state` message to the ESP32.
    *   The ESP32 responds with a `full_state_response` (TCP) containing all its current configurations and key state variables. The GUI uses this to populate its input fields and synchronize its understanding of the robot's state.
4.  **Sending Intents (UDP):**
    *   The GUI continuously (at a configurable frequency, e.g., 20Hz) sends `locomotion_intent`, `pose_adjust_intent`, and `centering_intent` messages via UDP to the ESP32 based on user keyboard input or DroidPad input.
    *   The ESP32 (`remote_control.cpp`) processes these intents to update its target movement and pose variables.
5.  **Sending Commands (TCP):**
    *   When the user clicks a button for a critical action (e.g., "Toggle Walk," "Apply Config," "Start Camera Stream"), the GUI sends a corresponding JSON command (e.g., `gait_command`, `config_update`, `camera_stream_control`) over TCP.
    *   The ESP32 processes the command and may send back an acknowledgement (e.g., `camera_stream_ack`) over TCP.
6.  **Receiving Telemetry:**
    *   The ESP32 sends subscribed telemetry data back to the GUI.
        *   Reliable but less frequent data (e.g., battery) is sent over TCP.
        *   Fast-updating data (e.g., actual velocities, pose) is sent over UDP to the GUI's telemetry listening port.
7.  **MJPEG Video Stream (Independent but often initiated via TCP command):**
    *   GUI sends a `camera_stream_control` (action: "start") command over TCP.
    *   ESP32 starts its MJPEG HTTP server via `streamer.cpp`.
    *   GUI's `MjpegStreamWorker` makes an HTTP GET request to `http://<robot_ip>:<mjpeg_port>/stream`.
    *   The worker receives and decodes JPEG frames, displaying them in the GUI.
    *   To stop, GUI sends `camera_stream_control` (action: "stop") over TCP, and the `MjpegStreamWorker` closes its HTTP connection.
8.  **Ping/Pong (TCP):** The GUI periodically sends `ping` messages over TCP. The ESP32 responds with `pong` messages, allowing the GUI to measure RTT and verify connection liveness.
9.  **Disconnect:** When the GUI disconnects, it may send a `disconnect_notice` (TCP) to the ESP32, allowing the ESP32 to clean up resources associated with that client (like telemetry subscriptions).

## 4. Key Software Modules Involved

*   **ESP32 Firmware:**
    *   `network_comms.cpp/.h`: Core TCP/UDP packet handling and JSON (de)serialization.
    *   `remote_control.cpp/.h`: High-level command processing, state management, telemetry, and discovery beacon logic.
    *   `streamer.cpp/.h`: MJPEG HTTP server for video.
*   **Python GUI:**
    *   `hexapod_comms_client.py`: Client-side TCP/UDP communication abstraction.
    *   `hexapod_gui.py`: Main application, including `TelemetryReceiverUDP`, `DiscoveryReceiverUDP`, and `MjpegStreamWorker` for handling specific network tasks, often in separate threads.