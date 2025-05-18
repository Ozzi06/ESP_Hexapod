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

        self.tcp_ping_timer = QTimer(self)  # Create the QTimer instance
        self.tcp_ping_timer.timeout.connect(self.send_ping_tcp)  # Connect its timeout signal to a method

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
            if local_ip == "0.0.0.0" or local_ip.startswith("127."):  # Handle loopback or unspec
                # Fallback if connected to self or unspecified network
                hostname = socket.gethostname()
                try:
                    # Try to get all IPs for the hostname and filter for non-loopback
                    all_ips = socket.getaddrinfo(hostname, None, socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_IP)
                    for res in all_ips:
                        ip_addr = res[4][0]
                        if not ip_addr.startswith("127."):
                            return ip_addr
                    # If only loopback found, use it
                    return socket.gethostbyname(hostname)
                except socket.gaierror:  # Fallback if getaddrinfo fails
                    return socket.gethostbyname(hostname)  # This might still be 127.0.0.1
            return local_ip
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror:
                return "0.0.0.0"  # Last resort

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

            PING_INTERVAL_MS = 2000
            self.tcp_ping_timer.start(PING_INTERVAL_MS)
            # print(f"[CommsClient DEBUG] TCP Ping Timer Started with interval {PING_INTERVAL_MS} ms.")

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
        # print("[CommsClient] TCP receive loop started.")
        should_disconnect_globally = False

        while self._is_tcp_connected and not self.tcp_receive_stop_event.is_set():
            try:
                if not self.tcp_socket:
                    # print("[CommsClient DEBUG] TCP socket is None in receive loop. Breaking.")
                    should_disconnect_globally = True
                    break

                data = self.tcp_socket.recv(2048)

                if not data:
                    print("[CommsClient] TCP connection closed by ESP32 (recv empty data).")
                    if self._is_tcp_connected:
                        self.tcp_disconnected_signal.emit("Connection closed by robot")
                    self._is_tcp_connected = False
                    should_disconnect_globally = True
                    break

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
            except Exception as e:
                if self._is_tcp_connected:
                    print(f"[CommsClient ERROR] TCP receive loop error: {e}")
                    self.tcp_disconnected_signal.emit(f"Receive Loop Error: {e}")
                self._is_tcp_connected = False
                should_disconnect_globally = True
                break

        # print(f"[CommsClient] TCP receive loop ended. is_connected: {self._is_tcp_connected}, stop_event: {self.tcp_receive_stop_event.is_set()}")
        self._is_tcp_connected = False
        self.tcp_receive_stop_event.set()

    def _process_tcp_buffer(self):
        while b'\n' in self.tcp_receive_buffer:
            line_bytes, self.tcp_receive_buffer = self.tcp_receive_buffer.split(b'\n', 1)
            line_str = line_bytes.decode('utf-8').strip()
            if line_str:
                try:
                    json_data = json.loads(line_str)
                    self.tcp_message_received_signal.emit(json_data)
                except json.JSONDecodeError as e:
                    print(f"[CommsClient ERROR] TCP JSON decode error: {e}. Data: '{line_str}'")
                except Exception as e:
                    print(f"[CommsClient ERROR] TCP processing error: {e}. Data: '{line_str}'")

    def disconnect_tcp(self):
        self.tcp_ping_timer.stop()
        # print("[CommsClient DEBUG] TCP Ping Timer Stopped.")

        if not self.tcp_receive_stop_event.is_set():
            self.tcp_receive_stop_event.set()

        self._is_tcp_connected = False

        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except Exception as e:
                pass
            self.tcp_socket = None

        if self.tcp_receive_thread and self.tcp_receive_thread.is_alive() and \
                threading.current_thread() != self.tcp_receive_thread:
            self.tcp_receive_thread.join(timeout=1.0)

        self.tcp_receive_thread = None
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
            return False
        try:
            json_string = json.dumps(message_dict) + "\n"
            self.tcp_socket.sendall(json_string.encode('utf-8'))
            return True
        except socket.timeout:
            print(f"[CommsClient ERROR] TCP sendall timed out.")
            self._is_tcp_connected = False
            self.tcp_disconnected_signal.emit("Send Timeout")
            self.disconnect_tcp()
            return False
        except Exception as e:
            print(f"[CommsClient ERROR] Failed to send TCP message: {e}")
            self._is_tcp_connected = False
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
                           gait_params=None, base_foot_positions=None,
                           acceleration_values=None, leg_geometry_abstract=None):  # TCP
        payload = {}
        if max_speeds: payload["max_speeds"] = max_speeds
        if pose_adjust_speeds: payload["pose_adjust_speeds"] = pose_adjust_speeds
        if gait_params: payload["gait_params"] = gait_params
        if base_foot_positions: payload["base_foot_positions_walk_cm"] = base_foot_positions
        if acceleration_values: payload["acceleration_values"] = acceleration_values
        if leg_geometry_abstract: payload["leg_geometry_abstract"] = leg_geometry_abstract

        if not payload:
            # print("[CommsClient DEBUG] send_config_update called but payload is empty.")
            return True  # No actual update to send
        # print(f"[CommsClient DEBUG] Sending config_update with payload: {payload}")
        return self._send_tcp({"type": "config_update", "source": "python_gui", "payload": payload})

    def send_pwm_reinitialize_command(self):  # TCP
        return self._send_tcp(
            {"type": "system_command", "source": "python_gui", "payload": {"action": "reinitialize_pwm"}})

    def send_client_settings(self, settings_payload: dict):  # TCP
        if 'udp_telemetry_config' in settings_payload:
            self.sent_udp_telemetry_path_setup = True
        return self._send_tcp({"type": "client_settings", "source": "python_gui", "payload": settings_payload})

    def send_request_full_state(self, reply_to_gui_ip: str):  # TCP
        return self._send_tcp({"type": "request_full_state", "source": "python_gui", "reply_to_ip": reply_to_gui_ip})

    def send_disconnect_notice(self):  # TCP
        if self.is_tcp_connected():
            # print("[CommsClient] Sending disconnect_notice to ESP32...")
            success = self._send_tcp({"type": "disconnect_notice", "source": "python_gui",
                                      "payload": {"source_ip": self.client_ip_for_esp_setup}})
            time.sleep(0.05)
            return success
        return False

    def send_ping_tcp(self):
        if self.is_tcp_connected():
            # print("[CommsClient DEBUG] Attempting to send TCP Ping to ESP32...")
            success = self._send_tcp({"type": "ping", "source": "python_gui", "payload": {"ts": time.time()}})
            # if success:
            # print("[CommsClient DEBUG] TCP Ping sent successfully.")
            # else:
            # print("[CommsClient DEBUG] TCP Ping FAILED to send.")
            # self.tcp_ping_timer.stop() # Already handled in _send_tcp on failure
        else:
            # print("[CommsClient DEBUG] TCP Ping: Not connected, stopping timer.")
            self.tcp_ping_timer.stop()

    def close(self):
        # print("[CommsClient] Closing communications...")
        try:
            self.send_locomotion_intent(0, 0, 0);
            time.sleep(0.01)
            self.send_pose_adjust_intent(0, 0, 0, 0, 0, 0);
            time.sleep(0.01)
            self.send_centering_intent(False, False)
        except Exception:
            pass

        self.disconnect_tcp()

        if self.udp_socket:
            try:
                self.udp_socket.close()
            except Exception:
                pass
            self.udp_socket = None
        # print("[CommsClient] Communications closed.")