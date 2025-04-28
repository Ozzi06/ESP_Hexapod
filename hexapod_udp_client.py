# hexapod_udp_client.py

import socket
import struct
import time
import math # For isnan if needed later

class HexapodUDPClient:
    """
    A client class to send FullControlPacket UDP datagrams to the ESP32 hexapod.

    Manages packet structure, sequence numbers, timestamps, and UDP sending.
    """

    # Define the packet structure format string (Little Endian)
    # Matches the C++ struct:
    # uint32 identifier      (I)
    # uint64 timestampMs     (Q)
    # uint32 sequenceNumber  (I)
    # uint8  controlFlags    (B)
    # float  velocityX       (f)
    # float  velocityY       (f)
    # float  velocityY       (f)
    # float  angularVelYaw   (f)
    # float  stepHeight      (f)
    # float  stepFrequency   (f)
    # float  dutyFactor      (f)
    # float  bodyPosX        (f)
    # float  bodyPosY        (f)
    # float  bodyPosZ        (f)
    # float  bodyOrientW     (f)
    # float  bodyOrientX     (f)
    # float  bodyOrientY     (f)
    # float  bodyOrientZ     (f)
    # Total: 73 bytes
    _PACKET_FORMAT = '<I Q I B fff f fff fff ffff'
    _PACKET_SIZE = 73 # Should match struct.calcsize(_PACKET_FORMAT)
    _PACKET_IDENTIFIER = 0xFEEDF00D

    # Bit masks for controlFlags
    _FLAG_WALK_RUNNING = (1 << 0)

    def __init__(self, target_ip: str, target_port: int):
        """
        Initializes the Hexapod UDP Client.

        Args:
            target_ip: The IP address of the ESP32 hexapod.
            target_port: The UDP port the ESP32 is listening on (e.g., 5005).
        """
        if struct.calcsize(self._PACKET_FORMAT) != self._PACKET_SIZE:
            raise ValueError(f"Packet format size mismatch! Expected {self._PACKET_SIZE}, got {struct.calcsize(self._PACKET_FORMAT)}")

        self.target_address = (target_ip, target_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sequence_number = 0

        # --- Initialize internal state variables for the packet ---
        # Metadata (managed internally)
        self._identifier = self._PACKET_IDENTIFIER
        self._timestamp_ms = 0
        # Control State
        self._walk_running = False
        # Locomotion Control (Walk Frame Relative)
        self.velocity_x = 0.0         # Sideways cm/s (+X Right)
        self.velocity_y = 0.0         # Forward/Backward cm/s (+Y Forward)
        self.velocity_z = 0.0         # Vertical cm/s (+Z Up) - Usually 0
        self.angular_velocity_yaw = 0.0 # Turning rad/s (+Yaw Left Turn)
        # Gait Parameters
        self.step_height = 3.0        # cm
        self.step_frequency = 1.0     # Hz
        self.duty_factor = 0.5        # 0.0 to 1.0
        # Body Pose Control (Walk Frame Relative)
        self.body_position_x = 0.0    # cm
        self.body_position_y = 0.0    # cm
        self.body_position_z = 10.0   # cm (Initial ride height)
        # Body Orientation (Quaternion, Walk Frame Relative) - Identity initially
        self.body_orientation_w = 1.0
        self.body_orientation_x = 0.0
        self.body_orientation_y = 0.0
        self.body_orientation_z = 0.0

        print(f"Hexapod UDP Client initialized for {target_ip}:{target_port}")
        print(f"Expected packet size: {self._PACKET_SIZE} bytes")

    def _get_timestamp_ms(self) -> int:
        """Returns the current time in milliseconds."""
        return int(time.time() * 1000)

    def set_walk_command(self, run: bool):
        """Sets the walk cycle running state."""
        self._walk_running = bool(run)

    def set_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """
        Sets the desired linear body velocity relative to the Walk Frame.
        Args:
            vx: Sideways velocity (cm/s, +X Right).
            vy: Forward/Backward velocity (cm/s, +Y Forward).
            vz: Vertical velocity (cm/s, +Z Up). Defaults to 0.
        """
        self.velocity_x = float(vx)
        self.velocity_y = float(vy)
        self.velocity_z = float(vz)

    def set_angular_velocity(self, yaw_rate: float):
        """
        Sets the desired angular body velocity (turning speed) relative to the Walk Frame.
        Args:
            yaw_rate: Turning speed around the Z axis (radians/s, +Yaw is Left Turn).
        """
        self.angular_velocity_yaw = float(yaw_rate)

    def set_gait_params(self, height: float, frequency: float, duty: float):
        """
        Sets the walk gait parameters.
        Args:
            height: Step height in cm.
            frequency: Step frequency in Hz.
            duty: Duty factor (0.01 to 0.99 recommended).
        """
        self.step_height = float(height)
        self.step_frequency = float(frequency)
        # Clamp duty factor to a reasonable range to match ESP32 receiver clamp
        self.duty_factor = max(0.01, min(0.99, float(duty)))

    def set_body_position(self, x: float, y: float, z: float):
        """
        Sets the desired body position offset relative to the Walk Frame origin.
        Args:
            x: X offset (cm, +X Right).
            y: Y offset (cm, +Y Forward).
            z: Z offset (ride height) (cm, +Z Up).
        """
        self.body_position_x = float(x)
        self.body_position_y = float(y)
        self.body_position_z = float(z)

    def set_body_orientation(self, w: float, x: float, y: float, z: float, normalize: bool = True):
        """
        Sets the desired body orientation relative to the Walk Frame axes using a quaternion.
        Args:
            w, x, y, z: Components of the quaternion.
            normalize: If True (default), normalize the quaternion before storing.
                       It's crucial that the quaternion sent is (close to) normalized.
        """
        qw, qx, qy, qz = float(w), float(x), float(y), float(z)
        if normalize:
            mag = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
            if mag > 1e-6: # Avoid division by zero
                qw /= mag
                qx /= mag
                qy /= mag
                qz /= mag
            else: # Invalid quaternion (magnitude zero), set to identity
                qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
                print("[Warning] Invalid zero-magnitude quaternion provided. Setting to identity.")

        self.body_orientation_w = qw
        self.body_orientation_x = qx
        self.body_orientation_y = qy
        self.body_orientation_z = qz

    def pack_data(self) -> bytes:
        """
        Packs the current state into the binary format for sending.
        Increments sequence number and updates timestamp.
        """
        self.sequence_number += 1
        self._timestamp_ms = self._get_timestamp_ms()

        control_flags = 0
        if self._walk_running:
            control_flags |= self._FLAG_WALK_RUNNING

        # --- IMPORTANT: Ensure arguments are passed to struct.pack in the EXACT order
        # defined by _PACKET_FORMAT ---
        try:
            packed_data = struct.pack(
                self._PACKET_FORMAT,
                # Metadata
                self._identifier,
                self._timestamp_ms,
                self.sequence_number,
                # Control State
                control_flags,
                # Locomotion Control
                self.velocity_x,
                self.velocity_y,
                self.velocity_z,
                self.angular_velocity_yaw,
                self.step_height,
                self.step_frequency,
                self.duty_factor,
                # Body Pose Control
                self.body_position_x,
                self.body_position_y,
                self.body_position_z,
                self.body_orientation_w,
                self.body_orientation_x,
                self.body_orientation_y,
                self.body_orientation_z
            )
            # Sanity check size after packing (optional)
            if len(packed_data) != self._PACKET_SIZE:
                 print(f"[Error] Packed data size unexpected! Expected {self._PACKET_SIZE}, got {len(packed_data)}")
                 # Decide how to handle this - maybe raise an error or return None?
                 # For now, just print the error. The send might fail or send corrupt data.

            return packed_data
        except struct.error as e:
            print(f"[Error] Failed to pack data: {e}")
            print("Current values:")
            print(f"  ID: {self._identifier}, TS: {self._timestamp_ms}, Seq: {self.sequence_number}")
            print(f"  Flags: {control_flags}")
            print(f"  LinVel: ({self.velocity_x}, {self.velocity_y}, {self.velocity_z})")
            print(f"  AngVel: {self.angular_velocity_yaw}")
            print(f"  Gait: ({self.step_height}, {self.step_frequency}, {self.duty_factor})")
            print(f"  Pos: ({self.body_position_x}, {self.body_position_y}, {self.body_position_z})")
            print(f"  Orient: ({self.body_orientation_w}, {self.body_orientation_x}, {self.body_orientation_y}, {self.body_orientation_z})")
            return None # Indicate packing failure

    def send(self):
        """
        Packs the current state and sends it as a UDP packet.
        """
        packed_bytes = self.pack_data()
        if packed_bytes:
            try:
                self.socket.sendto(packed_bytes, self.target_address)
            except socket.error as e:
                print(f"[Error] Socket error sending packet: {e}")
            # except Exception as e:
            #     print(f"[Error] Unexpected error sending packet: {e}")

    def close(self):
        """Closes the UDP socket."""
        print("Closing UDP socket.")
        self.socket.close()

# --- Example Usage ---
if __name__ == "__main__":
    # Replace with your ESP32's IP address
    ESP32_IP = "255.255.255.255" #just broadcast
    ESP32_PORT = 5005

    print("Starting Hexapod UDP Client Example...")
    client = HexapodUDPClient(ESP32_IP, ESP32_PORT)
    time.sleep(0.1)
    try:
        # Example: Make the hexapod walk forward slowly for 5 seconds
        print("Commanding walk forward...")
        client.set_walk_command(run=True)
        client.set_velocity(vx=0.0, vy=3.0, vz=0.0) # 3 cm/s forward
        client.set_angular_velocity(yaw_rate=0.0)
        client.set_body_position(x=0, y=0, z=12) # Set ride height to 12cm
        # Keep default orientation (identity quaternion)
        client.set_gait_params(height=3.0, frequency=1.5, duty=0.5)

        start_time = time.time()
        while time.time() - start_time < 5.0:
            client.send()
            print(f"Sent packet Seq={client.sequence_number}")
            time.sleep(0.55) # Send packets at ~20 Hz

        # Example: Make the hexapod turn left while stopped for 3 seconds
        print("Commanding turn left...")
        client.set_velocity(vx=0.0, vy=0.0, vz=0.0) # Stop linear movement
        client.set_angular_velocity(yaw_rate=0.5)  # 0.5 rad/s yaw rate (left turn)

        start_time = time.time()
        while time.time() - start_time < 3.0:
            client.send()
            print(f"Sent packet Seq={client.sequence_number}")
            time.sleep(0.05)

        # Example: Stop walking
        print("Commanding stop...")
        client.set_walk_command(run=False)
        client.set_velocity(vx=0.0, vy=0.0, vz=0.0)
        client.set_angular_velocity(yaw_rate=0.0)
        # Send stop command a few times to ensure it's received
        for _ in range(10):
            client.send()
            print(f"Sent packet Seq={client.sequence_number}")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nCaught Ctrl+C, stopping...")
    finally:
        # Ensure stop command is sent on exit
        print("Sending final stop command...")
        client.set_walk_command(run=False)
        client.set_velocity(vx=0.0, vy=0.0, vz=0.0)
        client.set_angular_velocity(yaw_rate=0.0)
        # Send a few times for good measure
        for _ in range(5):
            client.send()
            time.sleep(0.02)
        client.close()
        print("Client closed.")