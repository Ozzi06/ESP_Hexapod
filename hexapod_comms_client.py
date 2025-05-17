import socket
import json
import time
import threading
from PySide6.QtCore import QObject, Signal, QTimer  # For emitting signals to GUI

# Default ports for ESP32 (can be overridden by GUI inputs)
DEFAULT_ESP32_TCP_PORT = 5006
DEFAULT_ESP32_UDP_PORT = 5005


class HexapodCommsClient(QObject):
    # Signals for the GUI
    tcp_connected_signal = Signal()
    tcp_disconnected_signal = Signal(str)  # str for reason/error
    tcp_message_received_signal = Signal(dict)  # Parsed JSON from ESP32 via TCP

    # udp_message_received_signal = Signal(dict) # For UDP telemetry (handled by GUI's TelemetryReceiver)

    def __init__(self, robot_ip: str,
                 robot_tcp_port: int = DEFAULT_ESP32_TCP_PORT,
                 robot_udp_port: int = DEFAULT_ESP32_UDP_PORT,
                 gui_udp_listen_ip: str = "0.0.0.0",  # For ESP32 to send UDP telemetry back to
                 gui_udp_listen_port: int = 5007):
        super().__init__()
        self.robot_ip = robot_ip
        self.robot_tcp_addr = (robot_ip, robot_tcp_port)
        self.robot_udp_addr = (robot_ip, robot_udp_port)

        self.gui_udp_listen_ip_for_esp = gui_udp_listen_ip  # IP GUI tells ESP to send UDP to
        self.gui_udp_listen_port_for_esp = gui_udp_listen_port  # Port GUI tells ESP to send UDP to

        self.client_ip_for_esp_setup = self._get_local_ip_for_target(robot_ip)

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # UDP Broadcast setup (if robot_ip is broadcast)
        is_broadcast = robot_ip == "255.255.255.255" or robot_ip.endswith(".255")
        if is_broadcast:
            try:
                self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            except OSError as e:
                print(f"[CommsClient WARN] Could not enable SO_BROADCAST for UDP: {e}")

        self.tcp_socket = None
        self.tcp_receive_thread = None
        self.tcp_receive_stop_event = threading.Event()
        self._is_tcp_connected = False
        self.tcp_receive_buffer = b""

        self.tcp_ping_timer = QTimer(self) # Create the QTimer instance
        self.tcp_ping_timer.timeout.connect(self.send_ping_tcp) # Connect its timeout signal to a method

        # For sending initial client_settings for UDP telemetry path (only once per connection session)
        self.sent_udp_telemetry_path_setup = False

        print(f"HexapodCommsClient initialized. Target TCP: {self.robot_tcp_addr}, Target UDP: {self.robot_udp_addr}")
        print(f"  GUI IP for ESP's UDP telemetry: {self.client_ip_for_esp_setup}:{self.gui_udp_listen_port_for_esp}")

    def _get_local_ip_for_target(self, target_ip_str):
        # Attempts to find the local IP used to route to target_ip_str
        if target_ip_str == "0.0.0.0" or target_ip_str == "255.255.255.255":
            try:
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror:
                return "0.0.0.0"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
            s.settimeout(0.1)
            s.connect((target_ip_str, 1));
            local_ip = s.getsockname()[0];
            s.close()
            if local_ip == "0.0.0.0" or local_ip.startswith("127."):
                return socket.gethostbyname(socket.gethostname())
            return local_ip
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror:
                return "0.0.0.0"

    def get_local_ip_for_telemetry_setup(self):
        """Returns the determined local IP of the GUI machine for ESP setup."""
        return self.client_ip_for_esp_setup

    def connect_tcp(self) -> bool:
        if self._is_tcp_connected and self.tcp_socket:
            print("[CommsClient] TCP already connected.")
            return True

        self.disconnect_tcp()  # Ensure clean state

        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(3.0)  # Connection attempt timeout
            print(f"[CommsClient] Attempting TCP connection to {self.robot_tcp_addr}...")
            self.tcp_socket.connect(self.robot_tcp_addr)
            self.tcp_socket.settimeout(1.0)  # Timeout for subsequent recv calls

            self._is_tcp_connected = True
            self.sent_udp_telemetry_path_setup = False  # Reset for new session

            self.tcp_receive_stop_event.clear()
            self.tcp_receive_thread = threading.Thread(target=self._tcp_receive_loop, daemon=True)
            self.tcp_receive_thread.start()

            # Start the ping timer. Interval should be less than ESP32's TCP_CLIENT_TIMEOUT_MS.
            # If ESP32 timeout is 5s (5000ms), ping every 2-3s.
            # If ESP32 timeout is 30s (30000ms), ping every 10-15s.
            # Let's assume ESP32 timeout is currently 5s for your testing:
            PING_INTERVAL_MS = 2000 # e.g., 2 seconds
            self.tcp_ping_timer.start(PING_INTERVAL_MS)
            print(f"[CommsClient DEBUG] TCP Ping Timer Started with interval {PING_INTERVAL_MS} ms.")

            self.tcp_connected_signal.emit()
            print(f"[CommsClient] TCP connected successfully to {self.robot_tcp_addr}.")
            return True
        except socket.timeout:
            print(f"[CommsClient ERROR] TCP connection to {self.robot_tcp_addr} timed out.")
            self.disconnect_tcp()  # Cleanup
            self.tcp_disconnected_signal.emit(f"Connection Timeout to {self.robot_tcp_addr}")
            return False
        except Exception as e:
            print(f"[CommsClient ERROR] TCP connection failed: {e}")
            self.disconnect_tcp()  # Cleanup
            self.tcp_disconnected_signal.emit(f"Connection Error: {e}")
            return False

    def _tcp_receive_loop(self):
        print("[CommsClient] TCP receive loop started.")
        # ... (your heartbeat logging can stay) ...
        should_disconnect_globally = False  # Flag to indicate if main disconnect logic should run

        while self._is_tcp_connected and not self.tcp_receive_stop_event.is_set():
            try:
                if not self.tcp_socket:
                    print("[CommsClient DEBUG] TCP socket is None in receive loop. Breaking.")
                    should_disconnect_globally = True
                    break

                data = self.tcp_socket.recv(2048)

                if not data:
                    print("[CommsClient] TCP connection closed by ESP32 (recv empty data).")
                    if self._is_tcp_connected:  # Only emit if we thought we were connected
                        self.tcp_disconnected_signal.emit("Connection closed by robot")
                    self._is_tcp_connected = False  # Mark as disconnected
                    should_disconnect_globally = True  # Signal that full disconnect procedure is needed
                    break
                    # No self.disconnect_tcp() here

                self.tcp_receive_buffer += data
                self._process_tcp_buffer()

            except socket.timeout:
                continue
            except ConnectionResetError:
                print("[CommsClient ERROR] TCP ConnectionResetError in receive loop.")
                if self._is_tcp_connected:
                    self.tcp_disconnected_signal.emit("Connection Reset")
                self._is_tcp_connected = False
                should_disconnect_globally = True
                break
                # No self.disconnect_tcp() here
            except Exception as e:
                if self._is_tcp_connected:  # Only print/emit if we thought we were connected
                    print(f"[CommsClient ERROR] TCP receive loop error: {e}")
                    self.tcp_disconnected_signal.emit(f"Receive Loop Error: {e}")
                self._is_tcp_connected = False
                should_disconnect_globally = True
                break
                # No self.disconnect_tcp() here

        print(
            f"[CommsClient] TCP receive loop ended. is_connected: {self._is_tcp_connected}, stop_event: {self.tcp_receive_stop_event.is_set()}")

        # If the loop exited because the socket is bad or connection was lost,
        # ensure _is_tcp_connected is false and the stop event for this thread is set.
        # The main disconnect_tcp method, called from the GUI thread (or another thread),
        # will then handle the join.
        self._is_tcp_connected = False
        self.tcp_receive_stop_event.set()  # Ensure the event is set if loop exits unexpectedly

        # The actual self.disconnect_tcp() which does the join should be called from
        # the main thread in response to the tcp_disconnected_signal or when explicitly disconnecting.

    def _process_tcp_buffer(self):
        # ESP32 sends newline-terminated JSONs over TCP
        while b'\n' in self.tcp_receive_buffer:
            line_bytes, self.tcp_receive_buffer = self.tcp_receive_buffer.split(b'\n', 1)
            line_str = line_bytes.decode('utf-8').strip()
            if line_str:
                try:
                    json_data = json.loads(line_str)
                    # print(f"TCP RX (CommsClient): {json_data}") # Debug
                    self.tcp_message_received_signal.emit(json_data)
                except json.JSONDecodeError as e:
                    print(f"[CommsClient ERROR] TCP JSON decode error: {e}. Data: '{line_str}'")
                except Exception as e:
                    print(f"[CommsClient ERROR] TCP processing error: {e}. Data: '{line_str}'")

    def disconnect_tcp(self):
        # print("[CommsClient] disconnect_tcp() called.")
        self.tcp_ping_timer.stop() # Stop the ping timer on disconnect
        print("[CommsClient DEBUG] TCP Ping Timer Stopped.")

        if not self.tcp_receive_stop_event.is_set():  # Only set if not already set
            self.tcp_receive_stop_event.set()  # Signal receive thread to stop

        # Mark as not connected immediately.
        # This helps prevent race conditions where _send_tcp might try to use a closing socket.
        self._is_tcp_connected = False

        if self.tcp_socket:
            try:
                # self.tcp_socket.shutdown(socket.SHUT_RDWR) # Optional graceful shutdown
                self.tcp_socket.close()
            except Exception as e:
                # print(f"[CommsClient WARN] Error closing TCP socket: {e}")
                pass
            self.tcp_socket = None

        # Check if the current thread is the receive thread. If so, do not join.
        if self.tcp_receive_thread and self.tcp_receive_thread.is_alive() and \
                threading.current_thread() != self.tcp_receive_thread:  # Important check!
            # print("[CommsClient] Waiting for TCP receive thread to join...")
            self.tcp_receive_thread.join(timeout=1.0)  # Increased timeout slightly
            # if self.tcp_receive_thread.is_alive():
            #    print("[CommsClient WARN] TCP receive thread did not join.")

        self.tcp_receive_thread = None  # Clear the thread object
        self.tcp_receive_buffer = b""
        # print("[CommsClient] TCP disconnected (from disconnect_tcp method).")

    def is_tcp_connected(self) -> bool:
        return self._is_tcp_connected and self.tcp_socket is not None

    def _send_udp(self, message_dict: dict):
        try:
            json_string = json.dumps(message_dict)
            self.udp_socket.sendto(json_string.encode('utf-8'), self.robot_udp_addr)
        except Exception as e:
            print(f"[CommsClient ERROR] Failed to send UDP message: {e}\n  Message: {message_dict}")

    def _send_tcp(self, message_dict: dict) -> bool:
        if not self.is_tcp_connected():
            print("[CommsClient WARN] TCP not connected. Cannot send message.")
            # self.tcp_disconnected_signal.emit("Attempted send while disconnected") # Optionally signal GUI
            return False
        try:
            json_string = json.dumps(message_dict) + "\n"  # ESP32 expects newline terminated
            self.tcp_socket.sendall(json_string.encode('utf-8'))
            return True
        except socket.timeout:
            print(f"[CommsClient ERROR] TCP sendall timed out.")
            self._is_tcp_connected = False  # Assume connection is dead
            self.tcp_disconnected_signal.emit("Send Timeout")
            self.disconnect_tcp()
            return False
        except Exception as e:
            print(f"[CommsClient ERROR] Failed to send TCP message: {e}")
            self._is_tcp_connected = False  # Assume connection is dead
            self.tcp_disconnected_signal.emit(f"Send Error: {e}")
            self.disconnect_tcp()
            return False

    # --- Intent Sending Methods (UDP) ---
    def send_locomotion_intent(self, vx_factor, vy_factor, yaw_factor):
        payload = {
            "target_vx_factor": float(vx_factor), "target_vy_factor": float(vy_factor),
            "target_yaw_factor": float(yaw_factor)
        }
        self._send_udp({"type": "locomotion_intent", "source": "python_gui", "payload": payload})

    def send_pose_adjust_intent(self, offset_x, offset_y, offset_z, pitch, roll, body_yaw):
        payload = {
            "offset_x_active": float(offset_x), "offset_y_active": float(offset_y),
            "offset_z_active": float(offset_z), "pitch_active": float(pitch),
            "roll_active": float(roll), "body_yaw_active": float(body_yaw)
        }
        self._send_udp({"type": "pose_adjust_intent", "source": "python_gui", "payload": payload})

    def send_centering_intent(self, center_xy: bool, center_orientation: bool):
        payload = {"center_xy_active": bool(center_xy), "center_orientation_active": bool(center_orientation)}
        self._send_udp({"type": "centering_intent", "source": "python_gui", "payload": payload})

    # --- Command & Config Sending Methods (TCP) ---
    def send_gait_command(self, walk_active: bool):  # TCP
        return self._send_tcp(
            {"type": "gait_command", "source": "python_gui", "payload": {"walk_active": bool(walk_active)}})

    def send_config_update(self, max_speeds=None, pose_adjust_speeds=None,
                           gait_params=None, base_foot_positions=None):  # TCP
        payload = {}
        if max_speeds: payload["max_speeds"] = max_speeds
        if pose_adjust_speeds: payload["pose_adjust_speeds"] = pose_adjust_speeds
        if gait_params: payload["gait_params"] = gait_params
        if base_foot_positions: payload["base_foot_positions_walk_cm"] = base_foot_positions
        if not payload: return True  # No actual update to send
        return self._send_tcp({"type": "config_update", "source": "python_gui", "payload": payload})

    def send_pwm_reinitialize_command(self):  # TCP
        return self._send_tcp(
            {"type": "system_command", "source": "python_gui", "payload": {"action": "reinitialize_pwm"}})

    def send_client_settings(self, settings_payload: dict):  # TCP
        """ Sends client settings to ESP32, including telemetry subscriptions and UDP return path info. """
        # settings_payload is the fully formed payload for the client_settings message
        # e.g. {"udp_telemetry_config": {...}, "tcp_subscriptions_here": {...}}
        # The GUI constructs this.
        if 'udp_telemetry_config' in settings_payload:
            self.sent_udp_telemetry_path_setup = True  # Mark as sent for this session
        return self._send_tcp({"type": "client_settings", "source": "python_gui", "payload": settings_payload})

    def send_request_full_state(self, reply_to_gui_ip: str):  # TCP
        # reply_to_gui_ip is determined by the GUI and passed here
        return self._send_tcp({"type": "request_full_state", "source": "python_gui", "reply_to_ip": reply_to_gui_ip})

    def send_disconnect_notice(self):  # TCP
        if self.is_tcp_connected():
            # Send the notice then prepare to close on GUI side
            print("[CommsClient] Sending disconnect_notice to ESP32...")
            success = self._send_tcp({"type": "disconnect_notice", "source": "python_gui",
                                      "payload": {"source_ip": self.client_ip_for_esp_setup}})
            time.sleep(0.05)  # Give ESP a moment to process it
            return success
        return False

    def send_ping_tcp(self):
        if self.is_tcp_connected():
            print("[CommsClient DEBUG] Attempting to send TCP Ping to ESP32...")
            success = self._send_tcp({"type": "ping", "source": "python_gui", "payload": {"ts": time.time()}}) # Added timestamp to payload
            if success:
                print("[CommsClient DEBUG] TCP Ping sent successfully.")
            else:
                print("[CommsClient DEBUG] TCP Ping FAILED to send.")
                # If ping send fails, the _send_tcp method should have already handled
                # marking connection as dead and emitting disconnected signal.
                # We can also stop the timer here to be sure.
                self.tcp_ping_timer.stop()
        else:
            # This case should ideally not happen often if connect/disconnect logic is sound,
            # but good to have.
            print("[CommsClient DEBUG] TCP Ping: Not connected, stopping timer.")
            self.tcp_ping_timer.stop()

    def close(self):
        print("[CommsClient] Closing communications...")
        # Try to send stop intents via UDP
        try:
            self.send_locomotion_intent(0, 0, 0);
            time.sleep(0.01)
            self.send_pose_adjust_intent(0, 0, 0, 0, 0, 0);
            time.sleep(0.01)
            self.send_centering_intent(False, False)
        except Exception:
            pass  # Ignore errors during final stop sends

        self.disconnect_tcp()  # Handles TCP socket and thread

        if self.udp_socket:
            try:
                self.udp_socket.close()
            except Exception:
                pass
            self.udp_socket = None
        print("[CommsClient] Communications closed.")