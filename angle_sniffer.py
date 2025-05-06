import socket
import struct
import time
import math # For converting radians to degrees for display, if desired

# --- Configuration (MUST MATCH ESP32 SETUP) ---
LISTEN_IP = "0.0.0.0"      # Listen on all available network interfaces
LISTEN_PORT = 5006         # Must match angleBroadcastPort in ESP32 code
# Define MAX_SERVOS based on your robot_spec.h (e.g., LEG_COUNT * 3)
MAX_SERVOS = 18            # <--- CHANGE THIS to your robot's total servo count
# --- End Configuration ---

# --- Calculated values ---
EXPECTED_ANGLES = MAX_SERVOS
BYTES_PER_FLOAT = 4        # Standard size for C float
EXPECTED_PACKET_SIZE = EXPECTED_ANGLES * BYTES_PER_FLOAT
# '<' means little-endian (common for ESP32), 'f' means float. Repeat 'f' for each angle.
STRUCT_FORMAT = '<' + 'f' * EXPECTED_ANGLES
PRINT_INTERVAL_S = 0.1 # How often to print received data (seconds)
DISPLAY_IN_DEGREES = True # Set to False to display in radians
# ---

print(f"Starting UDP Angle Listener on {LISTEN_IP}:{LISTEN_PORT}")
print(f"Expecting {EXPECTED_ANGLES} angles ({EXPECTED_PACKET_SIZE} bytes) per packet.")
print(f"Struct format: '{STRUCT_FORMAT}'")
print(f"Displaying angles in: {'Degrees' if DISPLAY_IN_DEGREES else 'Radians'}")

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Allow address reuse (optional, can help avoid "address already in use" errors)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bind the socket to the listen address and port
try:
    sock.bind((LISTEN_IP, LISTEN_PORT))
    # Set a timeout so the loop doesn't block forever if no data comes
    sock.settimeout(0.5) # seconds
except OSError as e:
    print(f"\nError binding socket: {e}")
    print(f"Is another program using port {LISTEN_PORT}? Exiting.")
    exit(1)

print("Socket bound successfully. Waiting for angle data...")

last_print_time = time.monotonic()
packet_count = 0
angles = [0.0] * EXPECTED_ANGLES # Initialize with zeros

try:
    while True:
        try:
            # Receive data
            data, addr = sock.recvfrom(1024) # Buffer size, 1024 should be plenty

            # --- Validation ---
            if len(data) == EXPECTED_PACKET_SIZE:
                packet_count += 1
                # --- Unpack the received bytes into floats ---
                # The result 'unpacked_angles' will be a tuple
                unpacked_angles = struct.unpack(STRUCT_FORMAT, data)

                # --- Optional: Convert to list or use tuple directly ---
                angles = list(unpacked_angles) # More convenient if modifying later

                # --- Periodic Printing ---
                current_time = time.monotonic()
                if current_time - last_print_time >= PRINT_INTERVAL_S:
                    # Format angles for printing
                    if DISPLAY_IN_DEGREES:
                        # Convert radians to degrees for display
                        display_angles = [math.degrees(a) for a in angles]
                        unit = "deg"
                        format_spec = "+6.1f" # Format: +123.4
                    else:
                        display_angles = angles
                        unit = "rad"
                        format_spec = "+5.3f" # Format: +1.234

                    # Create the string representation
                    angle_str = ", ".join([f"{a:{format_spec}}" for a in display_angles])
                    print(f"\rRecv {packet_count:<6}: [{angle_str}] ({unit}) from {addr[0]} ", end="")
                    last_print_time = current_time

            elif len(data) > 0: # Avoid printing warnings for zero-byte packets if they occur
                print(f"\n[Warning] Received packet of unexpected size {len(data)} from {addr[0]}. Expected {EXPECTED_PACKET_SIZE}. Ignoring.")
                # Optionally print first few bytes if debugging:
                # print(f"          Data (hex): {data[:16].hex()}...")


        except socket.timeout:
            # No data received within the timeout, just continue looping
            # Display a waiting message if nothing received for a while
            if time.monotonic() - last_print_time > 2.0: # e.g., 2 seconds
                 print("\rWaiting for data...          ", end="")
            pass # Continue loop
        except struct.error as e:
            print(f"\n[Error] Failed to unpack data: {e}. Data length: {len(data)}. Format: '{STRUCT_FORMAT}'.")
        except Exception as e:
             print(f"\n[Error] An unexpected error occurred: {e}")
             time.sleep(1) # Avoid spamming errors


except KeyboardInterrupt:
    print("\nCtrl+C detected. Shutting down listener.")
finally:
    sock.close()
    print("Socket closed.")