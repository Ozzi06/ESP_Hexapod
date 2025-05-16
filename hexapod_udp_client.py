# hexapod_udp_client.py
import socket
import json
import time
import math


class HexapodUDPClient:
    def __init__(self, target_ip: str, target_port: int, gui_telemetry_listen_port: int = 0):
        print(f"Initializing UDP Client for JSON communication (Intent/Config Model).")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        is_broadcast_address = target_ip == "255.255.255.255" or target_ip.endswith(".255")
        if is_broadcast_address:
            try:
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                print("  Socket configured for BROADCAST.")
            except OSError as e:
                print(f"[WARN] Could not enable SO_BROADCAST on socket: {e}.")

        self.target_address = (target_ip, target_port)
        self.gui_telemetry_listen_port = gui_telemetry_listen_port
        self.client_ip_for_telemetry = self._get_local_ip_for_target(
            target_ip if not is_broadcast_address else "8.8.8.8")  # Use a known public DNS for interface guess if target is broadcast

        self.telemetry_subscriptions = {  # Default subscriptions for client_settings
            "battery": {"enabled": True, "interval_ms": 1000},
            "robot_status": {"enabled": True, "interval_ms": 5000},
            "robot_state_actual": {"enabled": True, "interval_ms": 200},  # For smooth GUI updates
            "debug_foot_pos": {"enabled": False, "interval_ms": 200}
        }
        self.sent_initial_client_settings = False  # Flag to ensure settings are sent once
        print(f"Hexapod UDP Client (JSON) initialized. Target: {self.target_address}")

    def _get_local_ip_for_target(self, target_ip_str):
        # Attempts to find the local IP used to route to target_ip_str
        # This is not foolproof, especially with complex network setups or VPNs.
        # For telemetry, if this fails, the ESP32 won't know where to send back.
        # A more robust solution for GUI telemetry is for the GUI to know its own IP on the relevant LAN.
        if target_ip_str == "0.0.0.0" or target_ip_str == "255.255.255.255":  # Cannot connect to broadcast to find interface
            try:  # Try to get primary hostname IP
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror:
                print("[WARN] Could not get local IP via hostname for telemetry. Using 0.0.0.0")
                return "0.0.0.0"  # ESP32 might not be able to use this
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)  # Prevent long block
            s.connect((target_ip_str, 1))  # Dummy port
            local_ip = s.getsockname()[0]
            s.close()
            # If it somehow resolved to 0.0.0.0 or loopback, try hostname method
            if local_ip == "0.0.0.0" or local_ip.startswith("127."):
                return socket.gethostbyname(socket.gethostname())
            return local_ip
        except Exception as e:
            print(f"[WARN] Could not determine specific local IP for telemetry via connect trick: {e}.")
            try:  # Fallback to hostname
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror as e2:
                print(f"[WARN] Fallback gethostbyname also failed: {e2}. Using '0.0.0.0'")
                return "0.0.0.0"

    def _send_json_message(self, message_dict):
        # Ensure client_settings is sent at least once before other commands
        # if telemetry is configured to be used by the GUI.
        if not self.sent_initial_client_settings and \
                message_dict.get("type") != "client_settings" and \
                (self.gui_telemetry_listen_port > 0 or any(
                    sub.get("enabled", False) for sub in self.telemetry_subscriptions.values())):
            print("[INFO] Sending initial client_settings before other command.")
            self.send_client_settings()  # Send it now
            time.sleep(0.02)  # Small pause to allow ESP32 to process

        try:
            json_string = json.dumps(message_dict)
            self.socket.sendto(json_string.encode('utf-8'), self.target_address)
        except Exception as e:
            print(f"[ERROR] Failed to send JSON message: {e}\nMessage was: {message_dict}")

    def send_client_settings(self, subscriptions_override=None):
        """Sends client settings including telemetry requests and listener info."""
        current_subscriptions = subscriptions_override if subscriptions_override is not None else self.telemetry_subscriptions

        payload_content = {}
        if self.gui_telemetry_listen_port > 0:  # Only include target if port is valid
            payload_content["telemetry_target"] = {
                "ip": self.client_ip_for_telemetry,  # This IP should be this GUI machine's IP on the robot's network
                "port": self.gui_telemetry_listen_port
            }

        if current_subscriptions:  # Only include if there are subscriptions to set
            payload_content["requested_telemetry"] = current_subscriptions

        if not payload_content:  # If nothing to configure (no listen port, no subs)
            print("[INFO] No telemetry target or subscriptions to send in client_settings.")
            self.sent_initial_client_settings = True  # Mark as "handled"
            return

        message = {
            "type": "client_settings",
            "source": "python_gui",
            "payload": payload_content
        }
        self._send_json_message(message)
        self.sent_initial_client_settings = True
        print(f"Sent client_settings to {self.target_address} (Telemetry listen IP: {self.client_ip_for_telemetry})")

    # --- Intent Sending Methods ---
    def send_locomotion_intent(self, vx_factor, vy_factor, yaw_factor):
        payload = {
            "target_vx_factor": float(vx_factor),
            "target_vy_factor": float(vy_factor),
            "target_yaw_factor": float(yaw_factor)
        }
        message = {"type": "locomotion_intent", "source": "python_gui", "payload": payload}
        self._send_json_message(message)

    def send_pose_adjust_intent(self, offset_x_active, offset_y_active, offset_z_active,
                                pitch_active, roll_active, body_yaw_active):
        payload = {
            "offset_x_active": float(offset_x_active),
            "offset_y_active": float(offset_y_active),
            "offset_z_active": float(offset_z_active),
            "pitch_active": float(pitch_active),
            "roll_active": float(roll_active),
            "body_yaw_active": float(body_yaw_active)
        }
        message = {"type": "pose_adjust_intent", "source": "python_gui", "payload": payload}
        self._send_json_message(message)

    def send_centering_intent(self, center_xy: bool, center_orientation: bool):
        payload = {
            "center_xy_active": bool(center_xy),
            "center_orientation_active": bool(center_orientation)
        }
        message = {"type": "centering_intent", "source": "python_gui", "payload": payload}
        self._send_json_message(message)

    def send_gait_command(self, walk_active: bool):
        payload = {"walk_active": bool(walk_active)}
        message = {"type": "gait_command", "source": "python_gui", "payload": payload}
        self._send_json_message(message)

    # --- Configuration Sending Methods ---
    def send_config_update(self, max_speeds=None, pose_adjust_speeds=None,
                           gait_params=None, base_foot_positions=None):
        payload = {}
        if max_speeds is not None: payload["max_speeds"] = max_speeds
        if pose_adjust_speeds is not None: payload["pose_adjust_speeds"] = pose_adjust_speeds
        if gait_params is not None: payload["gait_params"] = gait_params
        if base_foot_positions is not None: payload["base_foot_positions_walk_cm"] = base_foot_positions

        if not payload:
            print("[INFO] send_config_update called with no data to send.")
            return

        message = {"type": "config_update", "source": "python_gui", "payload": payload}
        self._send_json_message(message)

    # --- System Command Methods ---
    def send_pwm_reinitialize_command(self):
        payload = {"action": "reinitialize_pwm"}
        message = {"type": "system_command", "source": "python_gui", "payload": payload}
        self._send_json_message(message)

    def close(self):
        print("Closing UDP client socket (JSON client). Sending stop intents.")
        # Try to send stop intents if socket is still usable
        try:
            self.send_locomotion_intent(0, 0, 0)
            time.sleep(0.01)  # Small delay between packets
            self.send_pose_adjust_intent(0, 0, 0, 0, 0, 0)
            time.sleep(0.01)
            self.send_centering_intent(False, False)  # Ensure centering is off
            time.sleep(0.01)
            self.send_gait_command(False)  # Stop walking
            time.sleep(0.05)  # Allow final packets to go out
        except Exception as e:
            print(f"[WARN] Error sending final stop commands during close: {e}")
        finally:
            self.socket.close()