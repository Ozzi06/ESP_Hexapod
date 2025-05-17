#include "network_comms.h"
#include "Arduino.h" // For Serial, millis, etc.

// --- Static Module Variables ---
static WiFiServer* tcp_server = nullptr;
static WiFiUDP udp_listener;

// TCP Client Management
static WiFiClient active_tcp_clients[MAX_TCP_CLIENTS];
static char client_read_buffers[MAX_TCP_CLIENTS][TCP_CLIENT_BUFFER_SIZE];
static int client_read_buffer_fills[MAX_TCP_CLIENTS] = {0};
static unsigned long last_tcp_data_time[MAX_TCP_CLIENTS] = {0}; // For client timeout
const unsigned long TCP_CLIENT_TIMEOUT_MS = 5000; // 5 seconds of inactivity

// Callbacks
static JsonPacketProcessorCallback global_json_processor_cb = nullptr;
static TcpClientAbruptDisconnectCallback global_disconnect_cb = nullptr;

// Logging flag (can be controlled externally if needed, or set here)
static bool enable_network_logging = false; // Set to true for verbose network logs

// --- Private Helper Functions ---

/**
 * @brief Processes the data buffer for a specific TCP client, attempting to parse JSON objects.
 * Implements brace-counting to find individual JSON objects in a stream.
 *
 * @param client_idx The index of the TCP client in the active_tcp_clients array.
 */
static void process_tcp_client_buffer(uint8_t client_idx) {
    if (!global_json_processor_cb || client_idx >= MAX_TCP_CLIENTS) {
        return;
    }

    char* buffer = client_read_buffers[client_idx];
    int& buffer_fill = client_read_buffer_fills[client_idx]; // Use reference
    int processed_offset = 0;

    while (processed_offset < buffer_fill) {
        int obj_start_idx = -1;
        int obj_end_idx = -1;
        int brace_count = 0;
        bool in_string = false;

        // Find the start of a JSON object '{'
        for (int i = processed_offset; i < buffer_fill; ++i) {
            if (buffer[i] == '{') {
                obj_start_idx = i;
                break;
            }
        }

        if (obj_start_idx == -1) {
            // No '{' found, all remaining data is not a start of JSON or is garbage
            if (enable_network_logging && buffer_fill > processed_offset) {
                 Serial.printf("[NetComms TCP %s] Discarding %d non-JSON-start bytes: '", active_tcp_clients[client_idx].remoteIP().toString().c_str(), buffer_fill - processed_offset);
                 Serial.write((uint8_t*)(buffer + processed_offset), buffer_fill - processed_offset);
                 Serial.println("'");
            }
            buffer_fill = 0; // Discard unprocessed data if no '{'
            return; // Exit, nothing more to process from current buffer
        }

        // Found a '{', now find its matching '}'
        for (int i = obj_start_idx; i < buffer_fill; ++i) {
            char current_char = buffer[i];
            if (current_char == '"') {
                // Basic string handling: ignore escaped quotes
                if (i > 0 && buffer[i-1] == '\\') {
                    // This is an escaped quote, do nothing
                } else {
                    in_string = !in_string;
                }
            }

            if (!in_string) {
                if (current_char == '{') {
                    brace_count++;
                } else if (current_char == '}') {
                    brace_count--;
                }
            }

            if (brace_count == 0 && i >= obj_start_idx) { // Check i >= obj_start_idx for the case of empty {} object
                obj_end_idx = i;
                break;
            }
        }

        if (obj_end_idx != -1) {
            // Found a complete JSON object
            int obj_len = obj_end_idx - obj_start_idx + 1;
            
            // Create a temporary null-terminated string for parsing
            char temp_json_str[obj_len + 1];
            strncpy(temp_json_str, buffer + obj_start_idx, obj_len);
            temp_json_str[obj_len] = '\0';

            if (enable_network_logging) {
                Serial.printf("[NetComms TCP %s] RX Raw Obj: %s\n", active_tcp_clients[client_idx].remoteIP().toString().c_str(), temp_json_str);
            }

            DynamicJsonDocument doc(1024); // Adjust size as needed, should match remote_control's expectation
            DeserializationError error = deserializeJson(doc, temp_json_str);

            if (error) {
                if (enable_network_logging) {
                    Serial.printf("[NetComms TCP %s] JSON Deserialization failed: %s. Object: %s\n",
                                  active_tcp_clients[client_idx].remoteIP().toString().c_str(), error.c_str(), temp_json_str);
                }
                // Error handling: For now, we assume this "object" was malformed and skip it.
                // More robust handling might try to find the *next* '{'.
            } else {
                // Successfully parsed
                global_json_processor_cb(doc, active_tcp_clients[client_idx].remoteIP(), active_tcp_clients[client_idx].remotePort(), true, active_tcp_clients[client_idx]);
            }
            processed_offset = obj_end_idx + 1; // Advance past this successfully processed/attempted object
        } else {
            // Incomplete JSON object (no matching '}' yet), wait for more data
            // Shift unprocessed data (from obj_start_idx) to the beginning of the buffer
            if (obj_start_idx > 0) { // Only shift if there was leading data before this partial object
                memmove(buffer, buffer + obj_start_idx, buffer_fill - obj_start_idx);
                buffer_fill -= obj_start_idx;
            }
            // if (enable_network_logging) {
            //    Serial.printf("[NetComms TCP %s] Incomplete JSON, waiting for more. Buffer fill: %d\n", active_tcp_clients[client_idx].remoteIP().toString().c_str(), buffer_fill);
            // }
            return; // Exit and wait for more data
        }
    } // while (processed_offset < buffer_fill)

    // If all data processed, clear buffer_fill
    if (processed_offset >= buffer_fill) {
        buffer_fill = 0;
    } else {
        // Should not happen if logic is correct, but as a fallback:
        // Shift remaining unparsed data to the beginning
        memmove(buffer, buffer + processed_offset, buffer_fill - processed_offset);
        buffer_fill -= processed_offset;
    }
}

/**
 * @brief Stops and cleans up a TCP client slot.
*/
static void cleanup_tcp_client_slot(uint8_t client_idx) {
    if (client_idx < MAX_TCP_CLIENTS && active_tcp_clients[client_idx]) {
        IPAddress ip = active_tcp_clients[client_idx].remoteIP();
        if (enable_network_logging) {
            Serial.printf("[NetComms TCP] Cleaning up client slot %d for IP %s\n", client_idx, ip.toString().c_str());
        }
        active_tcp_clients[client_idx].stop();
        // active_tcp_clients[client_idx] = WiFiClient(); // Reassign to default invalid client
        client_read_buffer_fills[client_idx] = 0;
        last_tcp_data_time[client_idx] = 0;
        // The global_disconnect_cb will be called by the handler if it's an abrupt disconnect
    }
}


// --- Public Function Implementations ---

bool network_comms_setup(uint16_t tcp_listen_port,
                         uint16_t udp_listen_port,
                         JsonPacketProcessorCallback json_processor_cb,
                         TcpClientAbruptDisconnectCallback disconnect_cb) {
    global_json_processor_cb = json_processor_cb;
    global_disconnect_cb = disconnect_cb;

    // TCP Server Setup
    if (tcp_server) { // Delete if already exists (e.g. re-setup)
        delete tcp_server;
        tcp_server = nullptr;
    }
    tcp_server = new WiFiServer(tcp_listen_port);
    if (!tcp_server) {
        Serial.println("[NetComms ERR] Failed to create TCP server object.");
        return false;
    }
    tcp_server->begin();
    tcp_server->setNoDelay(true); // Send TCP packets immediately
    Serial.printf("[NetComms] TCP Server started on port %u\n", tcp_listen_port);

    // UDP Listener Setup
    if (udp_listener.begin(udp_listen_port)) {
        Serial.printf("[NetComms] UDP Listener started on port %u\n", udp_listen_port);
    } else {
        Serial.println("[NetComms ERR] Failed to start UDP Listener!");
        // Continue, TCP might still work
    }

    // Initialize TCP client slots
    for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
        active_tcp_clients[i] = WiFiClient(); // Ensure they are invalid initially
        client_read_buffer_fills[i] = 0;
        last_tcp_data_time[i] = 0;
    }
    
    Serial.printf("[NetComms] Local IP: %s\n", network_comms_get_local_ip().toString().c_str());
    return true;
}

void network_comms_handle() {
    if (!global_json_processor_cb) return; // Not properly setup

    unsigned long current_millis = millis();

    // 1. Handle TCP Server & Clients
    if (tcp_server) {
        // Check for new TCP connections
        if (tcp_server->hasClient()) {
            bool client_slot_found = false;
            for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
                if (!active_tcp_clients[i] || !active_tcp_clients[i].connected()) {
                    if(active_tcp_clients[i]) active_tcp_clients[i].stop(); // cleanup old if any

                    active_tcp_clients[i] = tcp_server->available();
                    if (active_tcp_clients[i]) {
                        active_tcp_clients[i].setNoDelay(true); // Low latency
                        client_read_buffer_fills[i] = 0; // Clear buffer for new client
                        last_tcp_data_time[i] = current_millis;
                        if (enable_network_logging) {
                            Serial.printf("[NetComms TCP] Client %d connected: %s\n", i, active_tcp_clients[i].remoteIP().toString().c_str());
                        }
                    }
                    client_slot_found = true;
                    break;
                }
            }
            if (!client_slot_found) {
                WiFiClient new_client = tcp_server->available(); // Get the client object
                if (enable_network_logging) {
                    Serial.printf("[NetComms TCP] Max clients reached. Rejecting %s\n", new_client.remoteIP().toString().c_str());
                }
                new_client.println("{\"type\":\"error\", \"message\":\"server_busy_max_clients\"}"); // Inform client
                new_client.stop(); // Reject
            }
        }

        // Process data from active TCP clients
        for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
            if (active_tcp_clients[i] && active_tcp_clients[i].connected()) {
                // Read available data
                int available_bytes = active_tcp_clients[i].available();
                if (available_bytes > 0) {
                    last_tcp_data_time[i] = current_millis; // Update activity time
                    int read_len = active_tcp_clients[i].read(
                        (uint8_t*)(client_read_buffers[i] + client_read_buffer_fills[i]),
                        TCP_CLIENT_BUFFER_SIZE - client_read_buffer_fills[i]
                    );

                    if (read_len > 0) {
                        client_read_buffer_fills[i] += read_len;
                        // Try to process the buffer immediately
                        process_tcp_client_buffer(i);
                    } else if (read_len < 0) { // Error on read
                         if (enable_network_logging) {
                            Serial.printf("[NetComms TCP %s] Read error on client %d.\n", active_tcp_clients[i].remoteIP().toString().c_str(), i);
                         }
                         if(global_disconnect_cb) global_disconnect_cb(active_tcp_clients[i].remoteIP());
                         cleanup_tcp_client_slot(i);
                    }
                }

                // Check for client timeout
                if (last_tcp_data_time[i] > 0 && (current_millis - last_tcp_data_time[i] > TCP_CLIENT_TIMEOUT_MS)) {
                    if (enable_network_logging) {
                        Serial.printf("[NetComms TCP %s] Client %d timed out.\n", active_tcp_clients[i].remoteIP().toString().c_str(), i);
                    }
                    if(global_disconnect_cb) global_disconnect_cb(active_tcp_clients[i].remoteIP());
                    cleanup_tcp_client_slot(i);
                }

            } else if (active_tcp_clients[i]) { // Client was valid but now disconnected
                // This case should ideally be caught by .connected() check or read error,
                // but as a fallback:
                IPAddress ip = active_tcp_clients[i].remoteIP(); // Get IP before cleanup if possible
                if (ip && ip != INADDR_NONE) { // Check if IP is valid
                     if (enable_network_logging) {
                        Serial.printf("[NetComms TCP %s] Client %d found disconnected.\n", ip.toString().c_str(), i);
                     }
                     if(global_disconnect_cb) global_disconnect_cb(ip);
                } else {
                     if (enable_network_logging) Serial.printf("[NetComms TCP] Client %d slot was occupied but now invalid/disconnected.\n", i);
                }
                cleanup_tcp_client_slot(i);
            }
        }
    }

    // 2. Handle UDP Packets
    int packet_size = udp_listener.parsePacket();
    if (packet_size > 0) {
        // Determine a reasonable maximum buffer size for UDP packets.
        // If actual packet_size exceeds this, we'll only read up to the buffer limit.
        char udp_packet_read_buffer[512]; // Max buffer for reading a single UDP JSON packet.
                                          // Adjust if your JSONs can be larger.
        
        // Read the packet data, ensuring not to overflow udp_packet_read_buffer.
        // remoteIP and remotePort must be called after parsePacket() and before reading the data.
        IPAddress remote_ip = udp_listener.remoteIP(); 
        uint16_t remote_port = udp_listener.remotePort();
        
        int len = udp_listener.read(udp_packet_read_buffer, 
                                    (packet_size < sizeof(udp_packet_read_buffer)) ? packet_size : (sizeof(udp_packet_read_buffer) - 1) );

        if (len > 0) {
            udp_packet_read_buffer[len] = '\0'; // Null-terminate the string

            DynamicJsonDocument doc(1024); // JSON doc for parsing. Size should accommodate largest expected UDP JSON.
            DeserializationError error = deserializeJson(doc, udp_packet_read_buffer);

            if (error) {
                if (enable_network_logging) {
                    Serial.printf("[NetComms UDP %s:%u] JSON Deserialization failed: %s. Raw: %s\n",
                                  remote_ip.toString().c_str(), remote_port, error.c_str(), udp_packet_read_buffer);
                }
            } else {
                // Successfully parsed
                if (enable_network_logging) {
                     Serial.printf("[NetComms UDP %s:%u] RX JSON successfully parsed.\n", remote_ip.toString().c_str(), remote_port);
                     // serializeJsonPretty(doc, Serial); Serial.println(); // Optional: Pretty print received UDP JSON
                }
                global_json_processor_cb(doc, remote_ip, remote_port, false, WiFiClient()); // Pass invalid client for UDP
            }
        } else if (len < 0) {
            // Error reading from UDP socket, though parsePacket indicated data.
            if (enable_network_logging) {
                Serial.printf("[NetComms UDP %s:%u] Error reading UDP packet (code %d) after parsePacket indicated size %d.\n",
                              remote_ip.toString().c_str(), remote_port, len, packet_size);
            }
        }
        // If len == 0, parsePacket() might have been misleading or packet was empty.
        // The UDP stack should handle moving to the next packet on the next call to parsePacket().
        // No explicit flush of udp_listener is typically needed here as parsePacket/read manage the current packet.
    }
}

bool network_comms_send_json_to_ip_tcp(IPAddress target_ip, const JsonDocument& doc) {
    if (!target_ip || target_ip == INADDR_NONE) return false;

    for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
        if (active_tcp_clients[i] && active_tcp_clients[i].connected() && active_tcp_clients[i].remoteIP() == target_ip) {
            String json_string;
            serializeJson(doc, json_string);
            
            if (enable_network_logging) {
                Serial.printf("[NetComms TCP %s] TX JSON: %s\n", target_ip.toString().c_str(), json_string.c_str());
            }
            // TCP uses println to ensure newline termination, assuming clients read line-by-line.
            size_t sent_bytes = active_tcp_clients[i].println(json_string);
            // active_tcp_clients[i].flush(); // Optional: can add latency
            
            if (sent_bytes == (json_string.length() + 2)) { // +2 for \r\n from println
                return true;
            } else {
                // If sent_bytes is 0, it's a more definite error or client already disconnected.
                // If sent_bytes > 0 but less than expected, it's a partial send.
                if (sent_bytes > 0 && sent_bytes < (json_string.length() + 2)) {
                    // Partial send occurred
                    if (enable_network_logging) { // Make sure enable_network_logging is true for this to print
                        Serial.printf("[NetComms TCP %s] WARNING: Partial send (%u of %u expected). Data might still arrive. Not disconnecting immediately.\n", 
                                      target_ip.toString().c_str(), sent_bytes, json_string.length()+2);
                    }
                    // Let's assume TCP will eventually deliver or timeout naturally.
                    // Returning true suggests the operation was initiated.
                    return true; 
                } else { 
                    // sent_bytes == 0 or some other error condition from println
                    // This is a more critical failure.
                    if (enable_network_logging) { // Make sure enable_network_logging is true for this to print
                        Serial.printf("[NetComms TCP %s] Send error (sent %u of %u expected). Disconnecting client.\n", 
                                      target_ip.toString().c_str(), sent_bytes, json_string.length()+2);
                    }
                    if(global_disconnect_cb) global_disconnect_cb(active_tcp_clients[i].remoteIP());
                    cleanup_tcp_client_slot(i);
                    return false;
                }
            }
        }
    }
    if (enable_network_logging) {
        Serial.printf("[NetComms TCP] No active client for IP %s to send JSON.\n", target_ip.toString().c_str());
    }
    return false;
}

bool network_comms_send_json_to_ip_port_udp(IPAddress target_ip, uint16_t target_port, const JsonDocument& doc) {
    if (!target_ip || target_ip == INADDR_NONE || target_port == 0) return false;

    String json_string;
    serializeJson(doc, json_string);

    if (udp_listener.beginPacket(target_ip, target_port)) {
        udp_listener.print(json_string);
        if (udp_listener.endPacket()) {
            if (enable_network_logging) {
                // Serial.printf("[NetComms UDP %s:%u] TX JSON: %s\n", target_ip.toString().c_str(), target_port, json_string.c_str()); // Too verbose
            }
            return true;
        } else {
            if (enable_network_logging) {
                 Serial.printf("[NetComms UDP %s:%u] endPacket() failed.\n", target_ip.toString().c_str(), target_port);
            }
        }
    } else {
         if (enable_network_logging) {
            Serial.printf("[NetComms UDP %s:%u] beginPacket() failed.\n", target_ip.toString().c_str(), target_port);
         }
    }
    return false;
}

void network_comms_close_client_by_ip(IPAddress target_ip) {
    if (!target_ip || target_ip == INADDR_NONE) return;
    for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
        if (active_tcp_clients[i] && active_tcp_clients[i].connected() && active_tcp_clients[i].remoteIP() == target_ip) {
            if (enable_network_logging) {
                Serial.printf("[NetComms TCP %s] Gracefully closing client %d by IP.\n", target_ip.toString().c_str(), i);
            }
            // No need to call global_disconnect_cb here as it's a graceful, requested close.
            // The caller (remote_control) handles subscription cleanup for graceful disconnects.
            cleanup_tcp_client_slot(i);
            return; // Assume only one client per IP
        }
    }
}

IPAddress network_comms_get_local_ip() {
    if (WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP();
    }
    return IPAddress(); // Invalid IP
}