import socket
import json
import time
import threading
from PySide6.QtCore import QObject, Signal, QTimer

DEFAULT_ESP32_TCP_PORT = 5006
DEFAULT_ESP32_UDP_PORT = 5005
DEFAULT_ESP32_MJPEG_PORT = 81 # For camera stream
PING_INTERVAL_MS = 2000
PONG_TIMEOUT_MS = 3000  # Should be > PING_INTERVAL_MS slightly, or independent


class HexapodCommsClient(QObject):
    tcp_connected_signal = Signal()
    tcp_disconnected_signal = Signal(str)
    tcp_message_received_signal = Signal(dict)
    rtt_updated_signal = Signal(float)  # New signal for RTT in ms

    def __init__(self, robot_ip: str,
                 robot_tcp_port: int = DEFAULT_ESP32_TCP_PORT,
                 robot_udp_port: int = DEFAULT_ESP32_UDP_PORT,
                 gui_udp_listen_ip: str = "0.0.0.0",
                 gui_udp_listen_port: int = 5007):
        super().__init__()
        self.robot_ip = robot_ip
        self.robot_tcp_addr = (robot_ip, robot_tcp_port)
        self.robot_udp_addr = (robot_ip, robot_udp_port)

        self.gui_udp_listen_ip_for_esp = gui_udp_listen_ip
        self.gui_udp_listen_port_for_esp = gui_udp_listen_port

        self.client_ip_for_esp_setup = self._get_local_ip_for_target(robot_ip)

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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

        self.tcp_ping_timer = QTimer(self)
        self.tcp_ping_timer.timeout.connect(self.send_ping_tcp)

        self.pong_timeout_timer = QTimer(self)  # For pong timeout
        self.pong_timeout_timer.setSingleShot(True)
        self.pong_timeout_timer.timeout.connect(self.handle_pong_timeout)
        self.last_ping_orig_ts = 0  # Timestamp when the last ping was sent

        self.sent_udp_telemetry_path_setup = False

        print(f"HexapodCommsClient initialized. Target TCP: {self.robot_tcp_addr}, Target UDP: {self.robot_udp_addr}")
        print(f"  GUI IP for ESP's UDP telemetry: {self.client_ip_for_esp_setup}:{self.gui_udp_listen_port_for_esp}")

    def _get_local_ip_for_target(self, target_ip_str):
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
                hostname = socket.gethostname()
                try:
                    all_ips = socket.getaddrinfo(hostname, None, socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_IP)
                    for res in all_ips:
                        ip_addr = res[4][0]
                        if not ip_addr.startswith("127."):
                            return ip_addr
                    return socket.gethostbyname(hostname)
                except socket.gaierror:
                    return socket.gethostbyname(hostname) # Fallback
            return local_ip
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror:
                return "0.0.0.0"

    def get_local_ip_for_telemetry_setup(self):
        return self.client_ip_for_esp_setup

    def connect_tcp(self) -> bool:
        if self._is_tcp_connected and self.tcp_socket:
            print("[CommsClient] TCP already connected.")
            return True

        self.disconnect_tcp()

        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(3.0)
            print(f"[CommsClient] Attempting TCP connection to {self.robot_tcp_addr}...")
            self.tcp_socket.connect(self.robot_tcp_addr)
            self.tcp_socket.settimeout(1.0)

            self._is_tcp_connected = True
            self.sent_udp_telemetry_path_setup = False

            self.tcp_receive_stop_event.clear()
            self.tcp_receive_thread = threading.Thread(target=self._tcp_receive_loop, daemon=True)
            self.tcp_receive_thread.start()

            self.tcp_ping_timer.start(PING_INTERVAL_MS)

            self.tcp_connected_signal.emit()
            print(f"[CommsClient] TCP connected successfully to {self.robot_tcp_addr}.")
            return True
        except socket.timeout:
            print(f"[CommsClient ERROR] TCP connection to {self.robot_tcp_addr} timed out.")
            self.disconnect_tcp()
            self.tcp_disconnected_signal.emit(f"Connection Timeout to {self.robot_tcp_addr}")
            return False
        except Exception as e:
            print(f"[CommsClient ERROR] TCP connection failed: {e}")
            self.disconnect_tcp()
            self.tcp_disconnected_signal.emit(f"Connection Error: {e}")
            return False

    def _tcp_receive_loop(self):
        while self._is_tcp_connected and not self.tcp_receive_stop_event.is_set():
            try:
                if not self.tcp_socket:
                    break

                data = self.tcp_socket.recv(2048)

                if not data:
                    print("[CommsClient] TCP connection closed by ESP32 (recv empty data).")
                    if self._is_tcp_connected:
                        self.tcp_disconnected_signal.emit("Connection closed by robot")
                    self._is_tcp_connected = False
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
                break
            except Exception as e:
                if self._is_tcp_connected:
                    print(f"[CommsClient ERROR] TCP receive loop error: {e}")
                    self.tcp_disconnected_signal.emit(f"Receive Loop Error: {e}")
                self._is_tcp_connected = False
                break

        self.tcp_ping_timer.stop()
        self.pong_timeout_timer.stop()
        self._is_tcp_connected = False
        self.tcp_receive_stop_event.set()

    def _process_tcp_buffer(self):
        while b'\n' in self.tcp_receive_buffer:
            line_bytes, self.tcp_receive_buffer = self.tcp_receive_buffer.split(b'\n', 1)
            line_str = line_bytes.decode('utf-8').strip()
            if line_str:
                try:
                    json_data = json.loads(line_str)
                    msg_type = json_data.get("type")

                    if msg_type == "pong":
                        self.pong_timeout_timer.stop()
                        payload = json_data.get("payload", {})
                        original_ts = payload.get("original_ts", 0)
                        if original_ts > 0:
                            rtt_s = time.time() - original_ts
                            self.rtt_updated_signal.emit(rtt_s * 1000.0)
                    else:
                        # All other messages are emitted for the GUI to handle
                        self.tcp_message_received_signal.emit(json_data)

                except json.JSONDecodeError as e:
                    print(f"[CommsClient ERROR] TCP JSON decode error: {e}. Data: '{line_str}'")
                except Exception as e:
                    print(f"[CommsClient ERROR] TCP processing error: {e}. Data: '{line_str}'")

    def disconnect_tcp(self):
        self.tcp_ping_timer.stop()
        self.pong_timeout_timer.stop()

        if not self.tcp_receive_stop_event.is_set():
            self.tcp_receive_stop_event.set()

        self._is_tcp_connected = False

        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except Exception as e:
                print(f"[CommsClient WARN] Error closing TCP socket: {e}")
            self.tcp_socket = None

        if self.tcp_receive_thread and self.tcp_receive_thread.is_alive() and \
                threading.current_thread() != self.tcp_receive_thread:
            self.tcp_receive_thread.join(timeout=1.0)
            if self.tcp_receive_thread.is_alive():
                print("[CommsClient WARN] TCP receive thread did not join.")

        self.tcp_receive_thread = None
        self.tcp_receive_buffer = b""

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
            self.tcp_disconnected_signal.emit("Send Timeout")
            self.disconnect_tcp()
            return False
        except Exception as e:
            print(f"[CommsClient ERROR] Failed to send TCP message: {e}")
            self.tcp_disconnected_signal.emit(f"Send Error: {e}")
            self.disconnect_tcp()
            return False

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

    def send_gait_command(self, walk_active: bool):
        return self._send_tcp(
            {"type": "gait_command", "source": "python_gui", "payload": {"walk_active": bool(walk_active)}})

    def send_config_update(self, max_speeds=None, pose_adjust_speeds=None,
                           gait_params=None, base_foot_positions=None,
                           acceleration_values=None, leg_geometry_abstract=None):
        payload = {}
        if max_speeds: payload["max_speeds"] = max_speeds
        if pose_adjust_speeds: payload["pose_adjust_speeds"] = pose_adjust_speeds
        if gait_params: payload["gait_params"] = gait_params
        if base_foot_positions: payload["base_foot_positions_walk_cm"] = base_foot_positions
        if acceleration_values: payload["acceleration_values"] = acceleration_values
        if leg_geometry_abstract: payload["leg_geometry_abstract"] = leg_geometry_abstract

        if not payload:
            return True # No actual update to send
        return self._send_tcp({"type": "config_update", "source": "python_gui", "payload": payload})

    def send_pwm_reinitialize_command(self):
        return self._send_tcp(
            {"type": "system_command", "source": "python_gui", "payload": {"action": "reinitialize_pwm"}})

    def send_client_settings(self, settings_payload: dict):
        if 'udp_telemetry_config' in settings_payload:
            self.sent_udp_telemetry_path_setup = True
        return self._send_tcp({"type": "client_settings", "source": "python_gui", "payload": settings_payload})

    def send_request_full_state(self, reply_to_gui_ip: str):
        return self._send_tcp({"type": "request_full_state", "source": "python_gui", "reply_to_ip": reply_to_gui_ip})

    def send_disconnect_notice(self):
        if self.is_tcp_connected():
            success = self._send_tcp({"type": "disconnect_notice", "source": "python_gui",
                                      "payload": {"source_ip": self.client_ip_for_esp_setup}})
            time.sleep(0.05)
            return success
        return False

    def send_ping_tcp(self):
        if self.is_tcp_connected():
            self.last_ping_orig_ts = time.time()
            ping_payload = {"ts": self.last_ping_orig_ts}
            success = self._send_tcp({"type": "ping", "source": "python_gui", "payload": ping_payload})
            if success:
                self.pong_timeout_timer.start(PONG_TIMEOUT_MS)
        else:
            self.tcp_ping_timer.stop()
            self.pong_timeout_timer.stop()

    def handle_pong_timeout(self):
        if self.is_tcp_connected():
            print("[CommsClient ERROR] Pong not received within timeout. Disconnecting.")
            self.tcp_disconnected_signal.emit("Ping Timeout")
            self.disconnect_tcp()

    # --- Camera Control Commands ---
    def send_camera_stream_control(self, action: str) -> bool:
        """
        Sends a command to start or stop the camera stream on the ESP32.
        :param action: "start" or "stop"
        :return: True if message was sent, False otherwise.
        """
        if action not in ["start", "stop"]:
            print(f"[CommsClient ERROR] Invalid camera stream action: {action}")
            return False
        payload = {"action": action}
        return self._send_tcp({"type": "camera_stream_control", "source": "python_gui", "payload": payload})

    def send_camera_config_update(self, resolution: str, quality: int, fps_limit: int) -> bool:
        """
        Sends camera configuration parameters to the ESP32.
        :param resolution: String identifier for resolution (e.g., "QVGA").
        :param quality: JPEG quality (0-63, lower is higher quality).
        :param fps_limit: Desired FPS limit (0 for no limit).
        :return: True if message was sent, False otherwise.
        """
        payload = {
            "resolution": resolution,
            "quality": int(quality),
            "fps_limit": int(fps_limit)
        }
        return self._send_tcp({"type": "camera_config_update", "source": "python_gui", "payload": payload})

    def close(self):
        try:
            self.send_locomotion_intent(0, 0, 0); time.sleep(0.01)
            self.send_pose_adjust_intent(0, 0, 0, 0, 0, 0); time.sleep(0.01)
            self.send_centering_intent(False, False)
        except Exception:
            pass # Ignore errors if already disconnected

        self.disconnect_tcp()

        if self.udp_socket:
            try:
                self.udp_socket.close()
            except Exception:
                pass
            self.udp_socket = None
        print("[CommsClient] Communications closed.")