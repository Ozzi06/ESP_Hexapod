import sys
import socket
import json
import time
import threading  # Not strictly needed for QThread but good for general knowledge
import requests
# from PIL import Image # QImage.loadFromData can often handle JPEGs directly
# import io # Not directly used with QImage.loadFromData
# import numpy as np # Not directly used with QImage.loadFromData

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QSpinBox, QGroupBox, QTextEdit,
    QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Slot, Signal, QObject, QThread
from PySide6.QtGui import QPixmap, QImage, QTextCursor

# --- Configuration ---
DEFAULT_ESP32_IP = "192.168.68.121"  # <<< CHANGE THIS IF NEEDED
DEFAULT_ESP32_TCP_PORT = 5006
DEFAULT_MJPEG_STREAM_PORT = 81  # As per streamer.cpp
MJPEG_BOUNDARY_BYTES = b"123456789000000000000987654321"  # From streamer.cpp PART_BOUNDARY

# --- Define Frame Size Strings at Module Level ---
FRAMESIZE_STR_QQVGA = "QQVGA"  # 160x120
FRAMESIZE_STR_HQVGA = "HQVGA"  # 240x176
FRAMESIZE_STR_QVGA = "QVGA"  # 320x240
FRAMESIZE_STR_CIF = "CIF"  # 400x296
FRAMESIZE_STR_VGA = "VGA"  # 640x480


# ... (CommandSenderWorker and MjpegStreamWorker classes remain the same) ...
class CommandSenderWorker(QObject):
    response_received_signal = Signal(str)
    error_signal = Signal(str)
    finished = Signal()  # Ensure finished signal is present

    def __init__(self, ip, port, command_dict):
        super().__init__()
        self.ip = ip
        self.port = port
        self.command_dict = command_dict

    @Slot()
    def run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5.0)
                s.connect((self.ip, self.port))

                json_string = json.dumps(self.command_dict) + "\n"
                s.sendall(json_string.encode('utf-8'))

                s.settimeout(5.0)
                response_buffer = b""
                # It's better to loop recv until a delimiter or connection close/timeout
                # For simple ACK, one recv might be enough, but robust clients loop.
                temp_data = s.recv(2048)  # Increased buffer for potentially larger ACKs
                if temp_data:
                    response_buffer += temp_data

                # Check if the last part of the buffer is a newline,
                # otherwise, try to receive more if the protocol expects it.
                # For this ACK, we assume one packet contains the full JSON + newline.

                if response_buffer:
                    # Try to decode only up to the first newline if multiple JSONs are accidentally sent
                    # or if there's trailing data.
                    try:
                        decoded_response = response_buffer.decode('utf-8')
                        json_end_index = decoded_response.find('\n')
                        if json_end_index != -1:
                            final_response_str = decoded_response[:json_end_index].strip()
                        else:
                            final_response_str = decoded_response.strip()

                        self.response_received_signal.emit(final_response_str)
                    except UnicodeDecodeError:
                        self.error_signal.emit("Failed to decode response (not UTF-8).")
                else:
                    self.error_signal.emit("No response or empty response from ESP32.")

        except socket.timeout:
            self.error_signal.emit(f"Timeout connecting or receiving from {self.ip}:{self.port}")
        except ConnectionRefusedError:
            self.error_signal.emit(f"Connection refused by {self.ip}:{self.port}")
        except Exception as e:
            self.error_signal.emit(f"Command sender error: {e}")
        finally:
            self.finished.emit()


class MjpegStreamWorker(QObject):
    new_frame_signal = Signal(QImage)
    stream_error_signal = Signal(str)
    finished = Signal()  # Ensure finished signal is present
    _is_running = False
    _stream_url = ""

    @Slot(str)
    def set_url(self, url):
        self._stream_url = url

    @Slot()
    def start_stream(self):
        if not self._stream_url:
            self.stream_error_signal.emit("Stream URL not set.")
            self.finished.emit()
            return

        self._is_running = True
        print(f"Worker: Attempting to connect to MJPEG stream: {self._stream_url}")
        session = requests.Session()  # Use a session for potential keep-alive
        try:
            # Add a User-Agent, some servers might require it
            headers = {'User-Agent': 'Python MJPEG Client'}
            response = session.get(self._stream_url, stream=True, timeout=(5, 10),
                                   headers=headers)  # (connect_timeout, read_timeout)
            response.raise_for_status()
            print("Worker: Connected to stream.")

            frame_data = b''
            full_boundary_bytes = b'--' + MJPEG_BOUNDARY_BYTES

            for chunk in response.iter_content(chunk_size=8192):  # Optimized chunk size
                if not self._is_running:
                    print("Worker: Stream stop requested.")
                    break

                frame_data += chunk

                while self._is_running:  # Process all complete frames in the current buffer
                    # Find the start of the boundary for the current part
                    boundary_start_pos = frame_data.find(full_boundary_bytes)
                    if boundary_start_pos == -1:
                        break  # Need more data to find a boundary

                    # Find the end of the boundary line (boundary + CRLF)
                    boundary_line_end_pos = frame_data.find(b'\r\n', boundary_start_pos)
                    if boundary_line_end_pos == -1:  # If not found, try with just LF
                        boundary_line_end_pos = frame_data.find(b'\n', boundary_start_pos)

                    if boundary_line_end_pos == -1:  # Still can't find end of boundary line
                        break  # Need more data

                    # The headers of the part start after the boundary line
                    headers_start_pos = boundary_line_end_pos + (
                        2 if frame_data[boundary_line_end_pos:boundary_line_end_pos + 2] == b'\r\n' else 1)

                    # Find the end of the headers (double CRLF or double LF)
                    jpeg_data_start_pos_after_headers = frame_data.find(b'\r\n\r\n', headers_start_pos)
                    header_delimiter_len = 4
                    if jpeg_data_start_pos_after_headers == -1:
                        jpeg_data_start_pos_after_headers = frame_data.find(b'\n\n', headers_start_pos)
                        header_delimiter_len = 2

                    if jpeg_data_start_pos_after_headers == -1:  # Can't find end of headers
                        break  # Need more data

                    # JPEG data begins after the header delimiter
                    actual_jpeg_data_start = jpeg_data_start_pos_after_headers + header_delimiter_len

                    # Find the start of the *next* boundary, which marks the end of the current JPEG data
                    next_boundary_start_pos = frame_data.find(full_boundary_bytes, actual_jpeg_data_start)

                    if next_boundary_start_pos == -1:  # Entire JPEG for this part not yet in buffer
                        break  # Need more data

                    # Extract the JPEG bytes
                    jpeg_bytes = frame_data[actual_jpeg_data_start:next_boundary_start_pos]

                    # Consume the processed part (from start of boundary to start of next boundary)
                    frame_data = frame_data[next_boundary_start_pos:]

                    if jpeg_bytes:
                        try:
                            q_image = QImage()
                            # QImage.loadFromData can directly load from bytes
                            if q_image.loadFromData(jpeg_bytes, "JPEG"):
                                if not q_image.isNull():
                                    self.new_frame_signal.emit(q_image)
                                # else:
                                #     print("Worker: Decoded QImage is null after loadFromData.") # Debug
                            # else:
                            #     print(f"Worker: QImage.loadFromData failed for {len(jpeg_bytes)} bytes.") # Debug
                        except Exception as e:
                            print(f"Worker: Error converting JPEG to QImage: {e}")

            if self._is_running and response:  # If loop exited for other reasons than stop_stream
                self.stream_error_signal.emit("Stream ended or connection lost.")
            print("Worker: Stream loop finished.")

        except requests.exceptions.ConnectionError as e:
            if self._is_running: self.stream_error_signal.emit(
                f"Stream Connection Error: {e}. Check IP/Port: {self._stream_url}")
        except requests.exceptions.Timeout:
            if self._is_running: self.stream_error_signal.emit(f"Stream Connection Timeout for {self._stream_url}")
        except requests.exceptions.RequestException as e:
            if self._is_running: self.stream_error_signal.emit(f"Stream request error: {e} for {self._stream_url}")
        finally:
            if 'response' in locals() and response:
                response.close()
            if 'session' in locals() and session:
                session.close()
            self._is_running = False  # Ensure flag is reset
            self.finished.emit()  # Signal that the worker's run method has completed
            print("Worker: Cleanup complete, finished emitted.")


class CameraTesterGUI(QMainWindow):
    # Make a signal to trigger URL set and start for the worker
    # This is better than directly calling worker methods from main thread if worker is in another thread
    _trigger_stream_worker_start_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Camera Stream Tester v1.1")
        self.setGeometry(100, 100, 800, 700)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self._command_thread = None
        self._command_worker = None
        self._stream_thread = None
        self._stream_worker = None

        self._init_ui()
        self.log_to_terminal("GUI Initialized. Configure IP and Port if needed.")

    def _init_ui(self):
        # Connection Group
        connection_group = QGroupBox("ESP32 Connection")
        connection_layout = QGridLayout(connection_group)
        connection_layout.addWidget(QLabel("ESP32 IP:"), 0, 0)
        self.ip_input = QLineEdit(DEFAULT_ESP32_IP)
        connection_layout.addWidget(self.ip_input, 0, 1)
        connection_layout.addWidget(QLabel("TCP Cmd Port:"), 1, 0)
        self.tcp_port_input = QLineEdit(str(DEFAULT_ESP32_TCP_PORT))
        self.tcp_port_input.setFixedWidth(100)
        connection_layout.addWidget(self.tcp_port_input, 1, 1)
        connection_layout.addWidget(QLabel("MJPEG Port:"), 2, 0)
        self.mjpeg_port_input = QLineEdit(str(DEFAULT_MJPEG_STREAM_PORT))
        self.mjpeg_port_input.setFixedWidth(100)
        connection_layout.addWidget(self.mjpeg_port_input, 2, 1)
        self.main_layout.addWidget(connection_group)

        # Camera Control Group
        control_group = QGroupBox("Camera Control")
        control_layout = QGridLayout(control_group)
        self.start_stream_button = QPushButton("Start Stream (View)")
        self.start_stream_button.clicked.connect(self.send_start_stream_command)  # This will also trigger viewing
        control_layout.addWidget(self.start_stream_button, 0, 0)
        self.stop_stream_button = QPushButton("Stop Stream (View & ESP)")
        self.stop_stream_button.clicked.connect(self.send_stop_stream_command)
        control_layout.addWidget(self.stop_stream_button, 0, 1)

        control_layout.addWidget(QLabel("Resolution:"), 1, 0)
        self.resolution_combo = QComboBox()
        # Use the module-level constants directly
        self.resolution_combo.addItems(
            [FRAMESIZE_STR_QQVGA, FRAMESIZE_STR_HQVGA, FRAMESIZE_STR_QVGA, FRAMESIZE_STR_CIF, FRAMESIZE_STR_VGA])
        self.resolution_combo.setCurrentText(FRAMESIZE_STR_QVGA)
        control_layout.addWidget(self.resolution_combo, 1, 1)

        control_layout.addWidget(QLabel("JPEG Quality (0-63):"), 2, 0)
        self.quality_spin = QSpinBox()
        self.quality_spin.setRange(0, 63)
        self.quality_spin.setValue(12)
        control_layout.addWidget(self.quality_spin, 2, 1)

        control_layout.addWidget(QLabel("FPS Limit:"), 3, 0)
        self.fps_spin = QSpinBox()
        self.fps_spin.setRange(0, 30)  # 0 for no limit
        self.fps_spin.setValue(10)
        control_layout.addWidget(self.fps_spin, 3, 1)

        self.apply_config_button = QPushButton("Apply Config to ESP32")
        self.apply_config_button.clicked.connect(self.send_camera_config_command)
        control_layout.addWidget(self.apply_config_button, 4, 0, 1, 2)
        self.main_layout.addWidget(control_group)

        # Video Display Group
        video_group = QGroupBox("Video Feed")
        video_layout = QVBoxLayout(video_group)
        self.video_label = QLabel("Video stream will appear here once 'Start Stream' is successful.")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(320, 240)  # Default QVGA size
        self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video_label.setStyleSheet("background-color: black; color: white; border: 1px solid gray;")
        video_layout.addWidget(self.video_label)
        self.main_layout.addWidget(video_group)
        self.main_layout.setStretchFactor(video_group, 1)

        # Log Area
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout(log_group)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setFixedHeight(150)
        log_layout.addWidget(self.log_area)
        self.main_layout.addWidget(log_group)

    def log_to_terminal(self, message):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_area.append(f"[{timestamp}] {message}")
        self.log_area.moveCursor(QTextCursor.MoveOperation.End)
        print(f"GUI LOG: {message}")

    def _send_generic_command(self, command_dict):
        esp_ip = self.ip_input.text()
        try:
            tcp_port = int(self.tcp_port_input.text())
        except ValueError:
            self.log_to_terminal("Error: Invalid TCP Command Port.")
            return

        # Crucial: Check if a previous thread is still in a state where it might be deleting
        # or if it's already None.
        if self._command_thread is not None and self._command_thread.isRunning():
            self.log_to_terminal("Warning: Previous command still processing. Please wait.")
            return

        # If thread exists but is not running, it might have finished and is pending deletion.
        # It's safer to just create a new one each time if the old one is not running.
        # However, the deleteLater mechanism should handle this.
        # The main issue is trying to reuse a thread that has emitted finished().

        # Always create new thread and worker for each command
        # This is a common and robust pattern for QThread.
        self.log_to_terminal(f"Sending command: {json.dumps(command_dict)}")

        self._command_thread = QThread(self)
        self._command_worker = CommandSenderWorker(esp_ip, tcp_port, command_dict)
        self._command_worker.moveToThread(self._command_thread)

        # Connections
        self._command_worker.response_received_signal.connect(self.handle_command_response)
        self._command_worker.error_signal.connect(self.handle_command_error)

        # When worker is done, it tells thread to quit
        self._command_worker.finished.connect(self._command_thread.quit)

        # When thread finishes, schedule both worker and thread for deletion
        # This prevents accessing deleted objects.
        self._command_thread.finished.connect(self._command_worker.deleteLater)
        self._command_thread.finished.connect(self.clear_command_thread_ref)  # New slot to clear ref

        self._command_thread.started.connect(self._command_worker.run)
        self._command_thread.start()

    @Slot() # New slot
    def clear_command_thread_ref(self):
        # This slot is called AFTER the thread has finished and deleteLater has been scheduled
        # for the thread itself. We can now safely nullify our Python reference to it.
        if self._command_thread == self.sender(): # Ensure it's the correct thread
            self._command_thread = None
            self._command_worker = None # Worker is also scheduled for deletion by this point
            self.log_to_terminal("Command thread and worker cleaned up.")

    @Slot(str)
    def handle_command_response(self, response_str):
        self.log_to_terminal(f"ESP32 Response: {response_str}")
        try:
            response_json = json.loads(response_str)
            self.log_to_terminal(f"Parsed ACK: {json.dumps(response_json, indent=2)}")

            msg_type = response_json.get("type")
            payload = response_json.get("payload", {})
            success = payload.get("success", False)

            if msg_type == "camera_stream_ack":
                action = payload.get("action_requested")
                if action == "start" and success:
                    self.log_to_terminal("Stream start command ACKed by ESP32. Attempting to view stream.")
                    self._start_mjpeg_display_thread()
                elif action == "stop" and success:
                    self.log_to_terminal("Stream stop command ACKed by ESP32.")
                    # Display thread should have already been signaled to stop by send_stop_stream_command
                    self.video_label.setText("Stream stopped.")
                    self.video_label.setStyleSheet("background-color: black; color: white;")
            elif msg_type == "camera_config_ack" and success:
                self.log_to_terminal("Camera config ACKed by ESP32.")
                # Could update local GUI fields from ack_payload["current_..."] if desired
            elif not success:
                self.log_to_terminal(f"ESP32 reported failure for {msg_type}: {payload.get('message', 'No details')}")


        except json.JSONDecodeError:
            self.log_to_terminal("ESP32 response was not valid JSON.")
        # Command worker's finished signal is connected to thread.quit, no need to call it here explicitly

    @Slot(str)
    def handle_command_error(self, error_msg):
        self.log_to_terminal(f"Command TCP Error: {error_msg}")
        # Command worker's finished signal is connected to thread.quit

    def send_start_stream_command(self):
        command = {
            "type": "camera_stream_control",
            "source": "python_gui_tester",
            "payload": {"action": "start"}
        }
        # This command, upon successful ACK, will trigger _start_mjpeg_display_thread
        self._send_generic_command(command)

    def send_stop_stream_command(self):
        self.log_to_terminal("User requested STOP stream.")
        # First, signal the local display thread to stop fetching
        self._stop_mjpeg_display_thread()

        # Then, send command to ESP32 to stop serving the stream
        command = {
            "type": "camera_stream_control",
            "source": "python_gui_tester",
            "payload": {"action": "stop"}
        }
        self._send_generic_command(command)
        # UI update for video_label will happen upon ACK or if display thread stops
        self.video_label.setText("Stream stop requested...")

    def send_camera_config_command(self):
        payload = {
            "resolution": self.resolution_combo.currentText(),
            "quality": self.quality_spin.value(),
            "fps_limit": self.fps_spin.value()
        }
        command = {
            "type": "camera_config_update",
            "source": "python_gui_tester",
            "payload": payload
        }
        self.log_to_terminal("Sending camera config. Note: This may stop/restart stream on ESP32 if it's active.")
        self._send_generic_command(command)

    def _start_mjpeg_display_thread(self):
        if self._stream_thread and self._stream_thread.isRunning():
            self.log_to_terminal("MJPEG display thread already running or starting.")
            return

        esp_ip = self.ip_input.text()
        try:
            mjpeg_port = int(self.mjpeg_port_input.text())
        except ValueError:
            self.log_to_terminal("Error: Invalid MJPEG Stream Port.")
            return

        stream_url = f"http://{esp_ip}:{mjpeg_port}/stream"
        self.log_to_terminal(f"Initializing MJPEG display thread for URL: {stream_url}")

        self._stream_thread = QThread(self)  # Parent to main window
        self._stream_worker = MjpegStreamWorker()
        self._stream_worker.moveToThread(self._stream_thread)

        # Connect the GUI's trigger signal to the worker's slot that sets the URL
        # and then the thread's started signal will call the worker's start_stream.
        # Or, more directly, pass URL at worker creation or via a dedicated slot before starting.
        # For simplicity here, worker gets URL via set_url method called by this triggering method.
        self._stream_worker.set_url(stream_url)

        self._stream_worker.new_frame_signal.connect(self.update_video_frame)
        self._stream_worker.stream_error_signal.connect(self.handle_stream_error)

        self._stream_worker.finished.connect(self._stream_thread.quit)
        self._stream_thread.finished.connect(self._stream_worker.deleteLater)
        self._stream_thread.finished.connect(self._stream_thread.deleteLater)

        self._stream_thread.started.connect(self._stream_worker.start_stream)

        self._stream_thread.start()
        self.video_label.setText("Connecting to MJPEG stream...")
        self.video_label.setStyleSheet("background-color: black; color: yellow;")

    def _stop_mjpeg_display_thread(self):
        if self._stream_worker:
            self.log_to_terminal("Signaling MJPEG worker to stop...")
            self._stream_worker.stop_stream()  # Signal worker to stop its loop

        if self._stream_thread and self._stream_thread.isRunning():
            self.log_to_terminal("Waiting for MJPEG display thread to quit...")
            # self._stream_thread.quit() # Worker's finished signal should trigger this
            if not self._stream_thread.wait(3000):  # Wait up to 3 seconds
                self.log_to_terminal("Warning: MJPEG display thread did not quit gracefully. Terminating.")
                self._stream_thread.terminate()
                self._stream_thread.wait()  # Wait after terminate
            self.log_to_terminal("MJPEG display thread stopped/terminated.")

        # Nullify to allow recreation
        self._stream_thread = None
        self._stream_worker = None
        self.video_label.setText("Stream display stopped.")
        self.video_label.setStyleSheet("background-color: black; color: white;")

    @Slot(QImage)
    def update_video_frame(self, q_image):
        if not q_image.isNull():
            pixmap = QPixmap.fromImage(q_image)
            self.video_label.setPixmap(pixmap.scaled(
                self.video_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            ))
        else:
            # This might indicate an issue with frame decoding or an empty frame signal
            # self.log_to_terminal("Received null QImage in update_video_frame.")
            pass

    @Slot(str)
    def handle_stream_error(self, error_msg):
        self.log_to_terminal(f"Stream Display Error: {error_msg}")
        self.video_label.setText(f"Stream Error:\n{error_msg}")
        self.video_label.setStyleSheet("background-color: darkred; color: white;")
        # Worker's finished signal should handle thread cleanup.
        # We ensure thread is cleaned up if it was running
        if self._stream_thread:
            self._stop_mjpeg_display_thread()

    def closeEvent(self, event):
        self.log_to_terminal("Closing application...")

        # Signal ESP32 to stop the stream
        # This is a fire-and-forget attempt, as GUI is closing.
        current_ip = self.ip_input.text()
        try:
            current_port = int(self.tcp_port_input.text())
            stop_command = {
                "type": "camera_stream_control",
                "source": "python_gui_tester_shutdown",
                "payload": {"action": "stop"}
            }
            # Create a temporary sender, non-threaded for quick attempt
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(0.5)  # Short timeout
                    s.connect((current_ip, current_port))
                    s.sendall((json.dumps(stop_command) + "\n").encode('utf-8'))
                self.log_to_terminal("Sent stream stop command to ESP32 (best effort).")
            except Exception as e:
                self.log_to_terminal(f"Could not send stop command on close: {e}")
        except ValueError:
            self.log_to_terminal("Invalid TCP port on close, cannot send stop command.")

        # Stop local threads
        self._stop_mjpeg_display_thread()

        if self._command_thread and self._command_thread.isRunning():
            self._command_thread.quit()
            if not self._command_thread.wait(1000):
                self.log_to_terminal("Command thread did not finish gracefully on close.")

        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraTesterGUI()
    window.show()
    sys.exit(app.exec())