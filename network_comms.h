#ifndef NETWORK_COMMS_H
#define NETWORK_COMMS_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// --- Configuration ---
const uint8_t MAX_TCP_CLIENTS = 3;       // Maximum simultaneous TCP clients
const uint16_t TCP_CLIENT_BUFFER_SIZE = 2048; // Max buffer per TCP client

// --- Callback Function Types ---
typedef void (*JsonPacketProcessorCallback)(const JsonDocument& doc, IPAddress source_ip, uint16_t source_port, bool is_tcp, WiFiClient tcp_client);
typedef void (*TcpClientAbruptDisconnectCallback)(IPAddress disconnected_client_ip);


// --- Public Function Declarations ---
bool network_comms_setup(uint16_t tcp_listen_port,
                         uint16_t udp_listen_port,
                         JsonPacketProcessorCallback json_processor_cb,
                         TcpClientAbruptDisconnectCallback disconnect_cb);
void network_comms_handle();
bool network_comms_send_json_to_ip_tcp(IPAddress target_ip, const JsonDocument& doc);
bool network_comms_send_json_to_ip_port_udp(IPAddress target_ip, uint16_t target_port, const JsonDocument& doc);
bool network_comms_send_broadcast_udp_json(uint16_t target_port, const JsonDocument& doc); // New
void network_comms_close_client_by_ip(IPAddress target_ip);
IPAddress network_comms_get_local_ip();
// bool network_comms_is_any_tcp_client_active(); // If needed for more complex LED


#endif // NETWORK_COMMS_H