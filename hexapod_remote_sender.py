import socket
import struct
import time
import sys

# --- Configuration ---
BROADCAST_IP = "255.255.255.255"
TARGET_PORT = 5005
PACKET_IDENTIFIER = 0xDEADBEEF

# --- Updated Packet Format ---
# '<' = little-endian
# I = unsigned int (identifier, 4 bytes)
# Q = unsigned long long (timestamp in milliseconds, 8 bytes) <--- CHANGED
# B = unsigned char (running flag, 1 byte)
# f = float (6x for velX, velY, velZ, height, freq, duty, 4 bytes each = 24 bytes)
PACKET_FORMAT = '<IQBffffff' # Changed 'f' timestamp to 'Q'
EXPECTED_SIZE = struct.calcsize(PACKET_FORMAT) # 4 + 8 + 1 + 24 = 37 bytes

# --- Global State ---
current_state = {
    "running": False,
    "vel_x": 0.0,
    "vel_y": 0.0,
    "vel_z": 0.0,
    "step_height": 2.0,
    "step_frequency": 0.75,
    "duty_factor": 0.5
}

def display_help():
    # Using the clear help format
    print("\n--- Remote Walk Cycle Control (UDP Broadcast w/ Millisecond Timestamp) ---")
    print(f"Broadcasting packets to port {TARGET_PORT} (Size: {EXPECTED_SIZE} bytes)")
    print("Ensure microcontroller is on the same network and listening on this port.")
    print("Commands:")
    print("  g        : Go / Start walk cycle")
    print("  b        : Break / Stop walk cycle")
    print("  x [val]  : Set X velocity (cm/s)")
    print("  y [val]  : Set Y velocity (cm/s)")
    # print("  z [val]  : Set Z velocity (cm/s)")
    print("  h [val]  : Set step Height (cm)")
    print("  f [val]  : Set step Frequency (Hz)")
    print("  d [val]  : Set Duty factor (0.0-1.0)")
    print("  s        : Show current state (doesn't send packet)")
    print("  send     : Re-send current state packet")
    print("  help     : Show this help message")
    print("  quit     : Exit program (sends stop packet)")
    print("-------------------------------------------------")
    print("NOTE: May require firewall adjustments or running as Admin on Windows.")
    print("-------------------------------------------------")

def pack_and_send(sock, state):
    """Packs the state with a millisecond timestamp (uint64) and broadcasts."""
    try:
        # Get current timestamp in MILLISECONDS as integer
        current_timestamp_ms = int(time.time() * 1000)

        packed_data = struct.pack(
            PACKET_FORMAT,
            PACKET_IDENTIFIER,
            current_timestamp_ms, # <<< Use integer milliseconds timestamp
            1 if state["running"] else 0,
            state["vel_x"],
            state["vel_y"],
            state["vel_z"],
            state["step_height"],
            state["step_frequency"],
            state["duty_factor"]
        )

        if len(packed_data) != EXPECTED_SIZE:
             print(f"[Error] Packed data size mismatch! Expected {EXPECTED_SIZE}, got {len(packed_data)}")
             return

        sock.sendto(packed_data, (BROADCAST_IP, TARGET_PORT))
        # No extra print here, main loop handles user feedback

    except socket.error as e:
        if isinstance(e, PermissionError) or (hasattr(e, 'winerror') and e.winerror == 10013):
             print(f"Socket Permission Error: {e}. Try running as administrator.")
        else:
             print(f"Socket Error: {e}")
    except Exception as e:
        print(f"Error packing/sending data: {e}")

# ===============================================================
# Restored Good Interface Main Loop (incorporating uint64 timestamp)
# ===============================================================
def main():
    """Main loop for user input and sending packets."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        print("UDP Socket created and configured for broadcast.")
    except PermissionError as e:
         print(f"Failed to configure socket for broadcast: {e}. Try running as administrator.")
         sys.exit(1)
    except socket.error as e:
        print(f"Failed to create or configure socket: {e}")
        sys.exit(1)

    display_help()

    while True:
        try:
            command_input = input("> ").strip().lower().split()
            if not command_input: continue
            command = command_input[0]
            args = command_input[1:]

            # Using the explicit if/elif structure user preferred
            if command == "quit":
                print("Setting state to stopped and sending final packet...")
                current_state["running"] = False
                pack_and_send(sock, current_state)
                time.sleep(0.1); break # Give time for packet
            elif command == "g":
                current_state["running"] = True
                print("Set state: Running = True. Sending packet...")
                pack_and_send(sock, current_state)
            elif command == "b":
                current_state["running"] = False
                print("Set state: Running = False. Sending packet...")
                pack_and_send(sock, current_state)
            elif command == "x" and args:
                try:
                    current_state["vel_x"] = float(args[0])
                    print(f"Set state: Velocity X = {current_state['vel_x']:.2f}. Sending packet...")
                    pack_and_send(sock, current_state)
                except ValueError: print("Invalid number format for X velocity.")
            elif command == "y" and args:
                try:
                    current_state["vel_y"] = float(args[0])
                    print(f"Set state: Velocity Y = {current_state['vel_y']:.2f}. Sending packet...")
                    pack_and_send(sock, current_state)
                except ValueError: print("Invalid number format for Y velocity.")
            elif command == "h" and args:
                 try:
                    current_state["step_height"] = float(args[0])
                    print(f"Set state: Step Height = {current_state['step_height']:.2f}. Sending packet...")
                    pack_and_send(sock, current_state)
                 except ValueError: print("Invalid number format for step height.")
            elif command == "f" and args:
                try:
                    current_state["step_frequency"] = float(args[0])
                    print(f"Set state: Step Frequency = {current_state['step_frequency']:.2f}. Sending packet...")
                    pack_and_send(sock, current_state)
                except ValueError: print("Invalid number format for step frequency.")
            elif command == "d" and args:
                 try:
                    df = max(0.01, min(0.99, float(args[0])))
                    current_state["duty_factor"] = df
                    print(f"Set state: Duty Factor = {current_state['duty_factor']:.2f}. Sending packet...")
                    pack_and_send(sock, current_state)
                 except ValueError: print("Invalid number format for duty factor.")
            elif command == "s":
                print("--- Current State (not sending) ---")
                print(f"  Timestamp: (Generated on send)")
                for key, value in current_state.items():
                    if isinstance(value, bool): print(f"  {key}: {value}")
                    else: print(f"  {key}: {value:.2f}")
                print("-----------------------------------")
            elif command == "send":
                 print("Re-sending current state...")
                 pack_and_send(sock, current_state)
            elif command == "help": display_help()
            else: print("Unknown command. Type 'help' for options.")
        except KeyboardInterrupt:
            print("\nExiting..."); current_state["running"] = False; print("Sending final stop packet..."); pack_and_send(sock, current_state); time.sleep(0.1); break
        except Exception as e: print(f"An unexpected error occurred: {e}")
    print("Closing socket."); sock.close()
# ===============================================================

if __name__ == "__main__":
    main()