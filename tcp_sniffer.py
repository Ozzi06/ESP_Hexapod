import socket
import threading
import time

# --- Configuration ---
LISTEN_HOST = '0.0.0.0'  # Listen on all available network interfaces
LISTEN_PORT = 5006  # Port for the TCP server to listen on
BUFFER_SIZE = 4096  # Increased buffer size for potentially larger packets

# --- Globals to manage client connection ---
client_socket_global = None
client_address_global = None
stop_event = threading.Event()  # To signal threads to stop


def get_local_ips():
    """Gets a list of local IP addresses of the machine."""
    local_ips = []
    try:
        # Get all interface addresses
        # This method is more comprehensive than the single connect-to-dummy-ip trick
        for info in socket.getaddrinfo(socket.gethostname(), None):
            # info is a tuple: (family, type, proto, canonname, sockaddr)
            # We are interested in IPv4 addresses (socket.AF_INET)
            if info[0] == socket.AF_INET:
                ip = info[4][0]  # sockaddr is (ip_address, port)
                if ip not in local_ips and not ip.startswith("127."):  # Avoid loopback
                    local_ips.append(ip)

        # Fallback if getaddrinfo doesn't give expected results (e.g., on some minimal systems)
        if not local_ips:
            try:
                # This is a common trick but might not list all IPs or the "best" one
                s_temp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s_temp.settimeout(0.1)
                s_temp.connect(('8.8.8.8', 1))  # connect to a known address
                ip_guess = s_temp.getsockname()[0]
                if ip_guess not in local_ips and not ip_guess.startswith("127."):
                    local_ips.append(ip_guess)
                s_temp.close()
            except Exception:
                pass  # Ignore if this fails

        if not local_ips:  # Absolute fallback
            local_ips.append("Could not determine a non-loopback local IP. Try '127.0.0.1' if app is on same machine.")

    except socket.gaierror:
        local_ips.append("Could not determine hostname or local IP via gethostname.")
    except Exception as e:
        local_ips.append(f"Error getting local IP: {e}")
    return local_ips


def handle_client_connection(conn, addr):
    """Handles communication with a connected client."""
    global client_socket_global, client_address_global

    # If another client was connected, notify and close old one (simple single-client handling)
    if client_socket_global and client_socket_global != conn:
        print(f"[SERVER] New client {addr} connected. Closing previous connection with {client_address_global}.")
        try:
            client_socket_global.close()
        except Exception as e:
            print(f"[SERVER_WARN] Error closing previous client socket: {e}")

    client_socket_global = conn
    client_address_global = addr
    print(f"\n[SERVER] Accepted connection from: {addr}")
    print(f"--- Start of session with {addr} ---")

    try:
        conn.settimeout(1.0)  # Set a timeout for recv so the loop can check stop_event
        while not stop_event.is_set():
            try:
                data = conn.recv(BUFFER_SIZE)
                if not data:
                    print(f"\n[SERVER] Client {addr} disconnected (received empty data).")
                    print(f"--- End of session with {addr} ---")
                    break

                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                print(f"\n[{timestamp} - From {addr[0]}:{addr[1]}] Received {len(data)} bytes:")

                # Try to decode as UTF-8 for readability
                try:
                    decoded_data = data.decode('utf-8')
                    # Print char by char to see control characters explicitly
                    printable_decoded = "".join(
                        [c if c.isprintable() or c in ['\n', '\r', '\t'] else f"\\x{ord(c):02x}" for c in decoded_data])
                    print(f"  UTF-8 Decoded: '{printable_decoded}'")
                except UnicodeDecodeError:
                    print(f"  UTF-8 Decode Error. Not valid UTF-8 text.")

                # Always print hex representation
                hex_data = data.hex(' ')
                print(f"  HEX Data: {hex_data}")
                print(f"  Raw Bytes: {data}")


            except socket.timeout:
                # Timeout allows the loop to check stop_event periodically
                continue
            except ConnectionResetError:
                print(f"\n[SERVER] Connection reset by {addr}.")
                print(f"--- End of session with {addr} ---")
                break
            except Exception as e:
                print(f"\n[SERVER_ERROR] Error during communication with {addr}: {e}")
                break
    except Exception as e:
        print(f"[SERVER_ERROR] Outer error with client {addr}: {e}")
    finally:
        print(f"[SERVER] Closing connection with {addr}.")
        if conn:
            conn.close()
        if client_socket_global == conn:  # Clear global if this was the active connection
            client_socket_global = None
            client_address_global = None


def start_tcp_sniffer_server():
    """Starts the TCP sniffer server."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse

    try:
        server_socket.bind((LISTEN_HOST, LISTEN_PORT))
        server_socket.listen(1)  # Listen for one client at a time for this sniffer

        print("--- TCP Sniffer Server ---")
        print(f"Listening for incoming TCP connections on port: {LISTEN_PORT}")

        local_ips = get_local_ips()
        if local_ips:
            print("Configure your mobile app to connect to one of these IP addresses (and port above):")
            for ip in local_ips:
                print(f"  IP: {ip}")
        else:
            print(
                "Could not determine local IP addresses automatically. You may need to find it manually (e.g. ipconfig/ifconfig).")
        print("Press Ctrl+C to stop the server.")
        print("-" * 30)

        while not stop_event.is_set():
            print("\n[SERVER] Waiting for a client (e.g., your mobile app) to connect...")
            try:
                server_socket.settimeout(1.0)  # Timeout for accept to check stop_event
                conn, addr = server_socket.accept()  # Blocking call with timeout

                # Handle client in a new thread so the main loop can be interrupted
                # and we can send data from the main thread.
                # For a simple sniffer, we might only want one client at a time.
                client_thread = threading.Thread(target=handle_client_connection, args=(conn, addr))
                client_thread.daemon = True  # Allows main program to exit cleanly
                client_thread.start()

            except socket.timeout:
                continue  # Go back to checking stop_event and waiting for connection
            except Exception as e:
                if not stop_event.is_set():  # Don't print error if we are stopping
                    print(f"[SERVER_ERROR] Error accepting connection: {e}")
                break  # Exit loop on other accept errors

    except KeyboardInterrupt:
        print("\n[SERVER] Shutdown signal received (Ctrl+C).")
    except Exception as e:
        print(f"[SERVER_FATAL_ERROR] Could not start server: {e}")
    finally:
        print("[SERVER] Stopping server and closing sockets...")
        stop_event.set()  # Signal all threads to stop
        if client_socket_global:
            try:
                client_socket_global.close()
            except:
                pass
        if server_socket:
            server_socket.close()
        print("[SERVER] Server stopped.")


if __name__ == "__main__":
    server_thread = threading.Thread(target=start_tcp_sniffer_server)
    server_thread.daemon = True
    server_thread.start()

    # Allow sending data from main thread to the connected client
    try:
        while not stop_event.is_set() and server_thread.is_alive():
            if client_socket_global and client_address_global:
                try:
                    # Check if input is available without blocking indefinitely (tricky cross-platform)
                    # For simplicity, using a timed input or just letting input() block
                    message_to_send = input(f"Enter message to send to {client_address_global} (or type 'exitapp'): ")
                    if message_to_send.lower() == 'exitapp':
                        print("Stopping server by user command.")
                        stop_event.set()
                        break
                    if client_socket_global:  # Check again, client might have disconnected
                        # Add newline, as many simple clients read line by line
                        client_socket_global.sendall((message_to_send + '\n').encode('utf-8'))
                        print(f"[SENT_TO_APP] '{message_to_send}\\n'")
                    else:
                        print("Client disconnected before message could be sent.")
                except EOFError:  # Happens if stdin is closed, e.g. in some non-interactive environments
                    print("Input stream closed. Server continues to run. Press Ctrl+C to stop.")
                    while not stop_event.is_set(): time.sleep(1)  # Keep alive until Ctrl+C
                    break
                except KeyboardInterrupt:  # Catch Ctrl+C specifically for input
                    print("\nInput interrupted. Stopping server.")
                    stop_event.set()
                    break
                except Exception as e:
                    if client_socket_global:  # Only print error if we thought we had a client
                        print(f"Error sending message: {e}")
                    # If client disconnected, handle_client_connection will clear client_socket_global
                    time.sleep(0.1)  # Small pause if there was an error before trying input again
            else:
                # If no client, wait a bit before checking again for input readiness
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[MAIN_THREAD] Shutdown signal received (Ctrl+C).")
        stop_event.set()

    if server_thread.is_alive():
        print("[MAIN_THREAD] Waiting for server thread to complete...")
        server_thread.join(timeout=2.0)  # Wait for the server thread to finish

    print("[MAIN_THREAD] Application finished.")