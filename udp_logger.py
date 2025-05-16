import socket
import argparse
import time # Import the time module

def udp_sniffer(listen_ip, listen_port, verbose=False):
    """
    Listens for UDP packets on a specified IP and port and logs them.
    """
    # Create a UDP socket
    # AF_INET for IPv4, SOCK_DGRAM for UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0) # Set a timeout so the loop can be interrupted more easily

    # Get local machine name and IP (for user info)
    try:
        hostname = socket.gethostname()
        # Try to get an IP address that is likely on the local network
        # This is a common way but not foolproof for multiple interfaces
        s_temp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s_temp.settimeout(0)
        try:
            # doesn't have to be reachable
            s_temp.connect(('10.254.254.254', 1))
            local_ip_guess = s_temp.getsockname()[0]
        except Exception:
            local_ip_guess = '127.0.0.1' # Fallback
        finally:
            s_temp.close()

        print(f"--- UDP Sniffer ---")
        print(f"Your hostname: {hostname}")
        print(f"Guessed local IP for app target: {local_ip_guess}")
        print(f"   (If this IP doesn't work, please check your system's network settings for the interface")
        print(f"    connected to the same Wi-Fi/network as your mobile app.)")

    except socket.gaierror:
        print("Could not determine hostname or local IP.")
    except Exception as e:
        print(f"Error getting local IP: {e}")


    # Bind the socket to the address and port
    try:
        sock.bind((listen_ip, listen_port))
        print(f"\nListening for UDP packets on {listen_ip}:{listen_port}...")
        if listen_ip == "0.0.0.0":
            print("   (Listening on all available network interfaces)")
    except socket.error as e:
        print(f"Error binding socket: {e}")
        print(f"Make sure the port {listen_port} is not already in use and you have permissions.")
        return

    print("Press Ctrl+C to stop the sniffer.")
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024) # Buffer size 1024 bytes
                source_ip, source_port = addr
                current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) # Get current time

                print(f"\n--- Packet Received ---")
                print(f"Timestamp: {current_time}")
                print(f"From IP: {source_ip}")
                print(f"From Port: {source_port}")
                print(f"Packet Size: {len(data)} bytes")

                print(f"Raw Data (bytes): {data}")

                try:
                    # Try decoding as UTF-8, replace errors if it's not valid UTF-8
                    decoded_data_utf8 = data.decode('utf-8', errors='replace')
                    print(f"Decoded Data (UTF-8): '{decoded_data_utf8}'")
                except Exception as e: # Catch any decoding error
                    print(f"Decoded Data (UTF-8): Error during decoding - {e}")


                if verbose:
                    print(f"Raw Data (hex): {data.hex()}")

            except socket.timeout:
                # This will now be caught every 1 second if no packet arrives
                # Allows Ctrl+C to be more responsive
                continue
            except Exception as e:
                print(f"Error during packet reception or processing: {e}")
                # Decide if you want to break or continue on other errors
                # break

    except KeyboardInterrupt:
        print("\nSniffer stopped by user.")
    finally:
        print("Closing socket.")
        sock.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple UDP Packet Sniffer")
    parser.add_argument(
        "--ip",
        type=str,
        default="0.0.0.0",
        help="The IP address to listen on (e.g., '0.0.0.0' for all interfaces, or a specific IP of this machine). Default: 0.0.0.0",
    )
    # Port is now hardcoded, but keeping arg for structure if needed later
    # parser.add_argument(
    #     "--port",
    #     type=int,
    #     required=True, # Will be defaulted
    #     help="The UDP port to listen on.",
    # )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose output (shows hex data)."
    )

    args = parser.parse_args()

    # Hardcode the port as requested
    target_port = 5005

    udp_sniffer(args.ip, target_port, args.verbose)