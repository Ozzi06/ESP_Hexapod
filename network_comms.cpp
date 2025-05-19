#include "network_comms.h"
#include "Arduino.h" // For Serial, millis, etc.

// --- Static Module Variables ---
static WiFiServer* tcp_server = nullptr;
static WiFiUDP udp_listener; // Used for both listening and sending UDP (including broadcasts)

// TCP Client Management
static WiFiClient active_tcp_clients[MAX_TCP_CLIENTS];
static char client_read_buffers[MAX_TCP_CLIENTS][TCP_CLIENT_BUFFER_SIZE];
static int client_read_buffer_fills[MAX_TCP_CLIENTS] = {0};
static unsigned long last_tcp_data_time[MAX_TCP_CLIENTS] = {0};
const unsigned long TCP_CLIENT_TIMEOUT_MS = 60000; // 60 seconds of inactivity

// Callbacks
static JsonPacketProcessorCallback global_json_processor_cb = nullptr;
static TcpClientAbruptDisconnectCallback global_disconnect_cb = nullptr;

// Logging flag
static bool enable_network_logging = false;

// --- Private Helper Functions ---
static void process_tcp_client_buffer(uint8_t client_idx) {
    if (!global_json_processor_cb || client_idx >= MAX_TCP_CLIENTS) {
        return;
    }

    char* buffer = client_read_buffers[client_idx];
    int& buffer_fill = client_read_buffer_fills[client_idx];
    int processed_offset = 0;

    while (processed_offset < buffer_fill) {
        int obj_start_idx = -1;
        int obj_end_idx = -1;
        int brace_count = 0;
        bool in_string = false;

        for (int i = processed_offset; i < buffer_fill; ++i) {
            if (buffer[i] == '{') {
                obj_start_idx = i;
                break;
            }
        }

        if (obj_start_idx == -1) {
            if (enable_network_logging && buffer_fill > processed_offset) {
                 Serial.printf("[NetComms TCP %s] Discarding %d non-JSON-start bytes: '", active_tcp_clients[client_idx].remoteIP().toString().c_str(), buffer_fill - processed_offset);
                 Serial.write((uint8_t*)(buffer + processed_offset), buffer_fill - processed_offset);
                 Serial.println("'");
            }
            buffer_fill = 0; 
            return; 
        }

        for (int i = obj_start_idx; i < buffer_fill; ++i) {
            char current_char = buffer[i];
            if (current_char == '"') {
                if (i > 0 && buffer[i-1] == '\\') {
                    // Escaped quote
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

            if (brace_count == 0 && i >= obj_start_idx) {
                obj_end_idx = i;
                break;
            }
        }

        if (obj_end_idx != -1) {
            int obj_len = obj_end_idx - obj_start_idx + 1;
            
            char temp_json_str[obj_len + 1];
            strncpy(temp_json_str, buffer + obj_start_idx, obj_len);
            temp_json_str[obj_len] = '\0';

            if (enable_network_logging) {
                Serial.printf("[NetComms TCP %s] RX Raw Obj: %s\n", active_tcp_clients[client_idx].remoteIP().toString().c_str(), temp_json_str);
            }

            DynamicJsonDocument doc(1024); 
            DeserializationError error = deserializeJson(doc, temp_json_str);

            if (error) {
                if (enable_network_logging) {
                    Serial.printf("[NetComms TCP %s] JSON Deserialization failed: %s. Object: %s\n",
                                  active_tcp_clients[client_idx].remoteIP().toString().c_str(), error.c_str(), temp_json_str);
                }
            } else {
                global_json_processor_cb(doc, active_tcp_clients[client_idx].remoteIP(), active_tcp_clients[client_idx].remotePort(), true, active_tcp_clients[client_idx]);
            }
            processed_offset = obj_end_idx + 1;
        } else {
            if (obj_start_idx > 0) { 
                memmove(buffer, buffer + obj_start_idx, buffer_fill - obj_start_idx);
                buffer_fill -= obj_start_idx;
            }
            return; 
        }
    } 

    if (processed_offset >= buffer_fill) {
        buffer_fill = 0;
    } else {
        memmove(buffer, buffer + processed_offset, buffer_fill - processed_offset);
        buffer_fill -= processed_offset;
    }
}

static void cleanup_tcp_client_slot(uint8_t client_idx) {
    if (client_idx < MAX_TCP_CLIENTS && active_tcp_clients[client_idx]) {
        IPAddress ip = active_tcp_clients[client_idx].remoteIP();
        if (enable_network_logging) {
            Serial.printf("[NetComms TCP] Cleaning up client slot %d for IP %s\n", client_idx, ip.toString().c_str());
        }
        active_tcp_clients[client_idx].stop();
        client_read_buffer_fills[client_idx] = 0;
        last_tcp_data_time[client_idx] = 0;
    }
}

// --- Public Function Implementations ---

bool network_comms_setup(uint16_t tcp_listen_port,
                         uint16_t udp_listen_port,
                         JsonPacketProcessorCallback json_processor_cb,
                         TcpClientAbruptDisconnectCallback disconnect_cb) {
    global_json_processor_cb = json_processor_cb;
    global_disconnect_cb = disconnect_cb;

    if (tcp_server) { 
        delete tcp_server;
        tcp_server = nullptr;
    }
    tcp_server = new WiFiServer(tcp_listen_port);
    if (!tcp_server) {
        Serial.println("[NetComms ERR] Failed to create TCP server object.");
        return false;
    }
    tcp_server->begin();
    tcp_server->setNoDelay(true);
    Serial.printf("[NetComms] TCP Server started on port %u\n", tcp_listen_port);

    if (udp_listener.begin(udp_listen_port)) {
        Serial.printf("[NetComms] UDP Listener started on port %u\n", udp_listen_port);
    } else {
        Serial.println("[NetComms ERR] Failed to start UDP Listener!");
    }

    for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
        active_tcp_clients[i] = WiFiClient(); 
        client_read_buffer_fills[i] = 0;
        last_tcp_data_time[i] = 0;
    }
    
    Serial.printf("[NetComms] Local IP: %s\n", network_comms_get_local_ip().toString().c_str());
    return true;
}

void network_comms_handle() {
    if (!global_json_processor_cb) return; 

    unsigned long current_millis = millis();

    if (tcp_server) {
        if (tcp_server->hasClient()) {
            bool client_slot_found = false;
            for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
                if (!active_tcp_clients[i] || !active_tcp_clients[i].connected()) {
                    if(active_tcp_clients[i]) active_tcp_clients[i].stop();

                    active_tcp_clients[i] = tcp_server->available();
                    if (active_tcp_clients[i]) {
                        active_tcp_clients[i].setNoDelay(true); 
                        client_read_buffer_fills[i] = 0; 
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
                WiFiClient new_client = tcp_server->available(); 
                if (enable_network_logging) {
                    Serial.printf("[NetComms TCP] Max clients reached. Rejecting %s\n", new_client.remoteIP().toString().c_str());
                }
                new_client.println("{\"type\":\"error\", \"message\":\"server_busy_max_clients\"}"); 
                new_client.stop(); 
            }
        }

        for (uint8_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
            if (active_tcp_clients[i] && active_tcp_clients[i].connected()) {
                int available_bytes = active_tcp_clients[i].available();
                if (available_bytes > 0) {
                    last_tcp_data_time[i] = current_millis; 
                    int read_len = active_tcp_clients[i].read(
                        (uint8_t*)(client_read_buffers[i] + client_read_buffer_fills[i]),
                        TCP_CLIENT_BUFFER_SIZE - client_read_buffer_fills[i]
                    );

                    if (read_len > 0) {
                        client_read_buffer_fills[i] += read_len;
                        process_tcp_client_buffer(i);
                    } else if (read_len < 0) { 
                         if (enable_network_logging) {
                            Serial.printf("[NetComms TCP %s] Read error on client %d.\n", active_tcp_clients[i].remoteIP().toString().c_str(), i);
                         }
                         if(global_disconnect_cb) global_disconnect_cb(active_tcp_clients[i].remoteIP());
                         cleanup_tcp_client_slot(i);
                    }
                }

                if (last_tcp_data_time[i] > 0 && (current_millis - last_tcp_data_time[i] > TCP_CLIENT_TIMEOUT_MS)) {
                    if (enable_network_logging) {
                        Serial.printf("[NetComms TCP %s] Client %d timed out.\n", active_tcp_clients[i].remoteIP().toString().c_str(), i);
                    }
                    if(global_disconnect_cb) global_disconnect_cb(active_tcp_clients[i].remoteIP());
                    cleanup_tcp_client_slot(i);
                }

            } else if (active_tcp_clients[i]) { 
                IPAddress ip = active_tcp_clients[i].remoteIP(); 
                if (ip && ip != INADDR_NONE) { 
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

    int packet_size = udp_listener.parsePacket();
    if (packet_size > 0) {
        char udp_packet_read_buffer[512]; 
        
        IPAddress remote_ip = udp_listener.remoteIP(); 
        uint16_t remote_port = udp_listener.remotePort();
        
        int len = udp_listener.read(udp_packet_read_buffer, 
                                    (packet_size < sizeof(udp_packet_read_buffer)) ? packet_size : (sizeof(udp_packet_read_buffer) - 1) );

        if (len > 0) {
            udp_packet_read_buffer[len] = '\0'; 

            DynamicJsonDocument doc(1024); 
            DeserializationError error = deserializeJson(doc, udp_packet_read_buffer);

            if (error) {
                if (enable_network_logging) {
                    Serial.printf("[NetComms UDP %s:%u] JSON Deserialization failed: %s. Raw: %s\n",
                                  remote_ip.toString().c_str(), remote_port, error.c_str(), udp_packet_read_buffer);
                }
            } else {
                if (enable_network_logging) {
                     Serial.printf("[NetComms UDP %s:%u] RX JSON successfully parsed.\n", remote_ip.toString().c_str(), remote_port);
                }
                global_json_processor_cb(doc, remote_ip, remote_port, false, WiFiClient()); 
            }
        } else if (len < 0) {
            if (enable_network_logging) {
                Serial.printf("[NetComms UDP %s:%u] Error reading UDP packet (code %d) after parsePacket indicated size %d.\n",
                              remote_ip.toString().c_str(), remote_port, len, packet_size);
            }
        }
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
            size_t sent_bytes = active_tcp_clients[i].println(json_string);
            
            if (sent_bytes == (json_string.length() + 2)) { 
                return true;
            } else {
                if (sent_bytes > 0 && sent_bytes < (json_string.length() + 2)) {
                    if (enable_network_logging) { 
                        Serial.printf("[NetComms TCP %s] WARNING: Partial send (%u of %u expected). Data might still arrive. Not disconnecting immediately.\n", 
                                      target_ip.toString().c_str(), sent_bytes, json_string.length()+2);
                    }
                    return true; 
                } else { 
                    if (enable_network_logging) { 
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
            // UDP sends are fire-and-forget, logging can be too verbose
            // if (enable_network_logging) {
            //     Serial.printf("[NetComms UDP %s:%u] TX JSON: %s\n", target_ip.toString().c_str(), target_port, json_string.c_str());
            // }
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

bool network_comms_send_broadcast_udp_json(uint16_t target_port, const JsonDocument& doc) {
    if (target_port == 0) return false;

    String json_string;
    serializeJson(doc, json_string);

    IPAddress broadcast_ip(255, 255, 255, 255); 

    // The udp_listener instance is already initialized (bound to a listening port).
    // It can also be used to send packets.
    if (udp_listener.beginPacket(broadcast_ip, target_port)) {
        udp_listener.print(json_string);
        if (udp_listener.endPacket()) {
            if (enable_network_logging) {
                 Serial.printf("[NetComms UDP BROADCAST :%u] TX JSON: %s\n", target_port, json_string.c_str());
            }
            return true;
        } else {
            if (enable_network_logging) {
                 Serial.printf("[NetComms UDP BROADCAST :%u] endPacket() failed.\n", target_port);
            }
        }
    } else {
         if (enable_network_logging) {
            Serial.printf("[NetComms UDP BROADCAST :%u] beginPacket() failed.\n", target_port);
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
            cleanup_tcp_client_slot(i);
            return; 
        }
    }
}

IPAddress network_comms_get_local_ip() {
    if (WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP();
    }
    return IPAddress(); 
}