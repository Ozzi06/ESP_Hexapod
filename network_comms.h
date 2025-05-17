#ifndef NETWORK_COMMS_H
#define NETWORK_COMMS_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// --- Configuration ---
const uint8_t MAX_TCP_CLIENTS = 3;       // Maximum simultaneous TCP clients (e.g., GUI + 2 Apps, or GUI + App + Debug)
const uint16_t TCP_CLIENT_BUFFER_SIZE = 2048; // Max buffer per TCP client for incoming concatenated JSONs
                                             // Should be large enough for a few typical JSONs from mobile app.

// --- Callback Function Types ---

/**
 * @brief Callback function type for when a complete JSON document has been parsed.
 *
 * @param doc The parsed JsonDocument.
 * @param source_ip The IP address of the sender.
 * @param source_port The port of the sender.
 * @param is_tcp True if the message came via TCP, false if via UDP.
 * @param tcp_client If is_tcp is true, this is the WiFiClient object for the connection (can be used for direct replies).
 *                   If is_tcp is false, this client object will be invalid/disconnected.
 */
typedef void (*JsonPacketProcessorCallback)(const JsonDocument& doc, IPAddress source_ip, uint16_t source_port, bool is_tcp, WiFiClient tcp_client);

/**
 * @brief Callback function type for notifying about an abrupt TCP client disconnection.
 *
 * @param disconnected_client_ip The IP address of the TCP client that disconnected abruptly.
 */
typedef void (*TcpClientAbruptDisconnectCallback)(IPAddress disconnected_client_ip);


// --- Public Function Declarations ---

/**
 * @brief Initializes the network communications module (TCP server and UDP listener).
 *
 * @param tcp_listen_port Port for the TCP server to listen on.
 * @param udp_listen_port Port for the UDP listener.
 * @param json_processor_cb Callback function to be invoked when a JSON packet is parsed.
 * @param disconnect_cb Callback function to be invoked on abrupt TCP client disconnection.
 * @return True if setup was successful, false otherwise.
 */
bool network_comms_setup(uint16_t tcp_listen_port,
                         uint16_t udp_listen_port,
                         JsonPacketProcessorCallback json_processor_cb,
                         TcpClientAbruptDisconnectCallback disconnect_cb);

/**
 * @brief Handles incoming TCP connections, TCP data, and UDP packets.
 * This function should be called repeatedly in the main loop.
 */
void network_comms_handle();

/**
 * @brief Sends a JSON document to a specific target IP address via TCP.
 * It will find an active TCP client connected from target_ip and send to it.
 * Assumes only one active TCP connection per IP address.
 *
 * @param target_ip The IP address of the recipient.
 * @param doc The JsonDocument to send. The document will be serialized with a newline.
 * @return True if the message was successfully sent (or queued by TCP stack), false otherwise
 *         (e.g., no active TCP client from that IP, send error).
 */
bool network_comms_send_json_to_ip_tcp(IPAddress target_ip, const JsonDocument& doc);

/**
 * @brief Sends a JSON document to a specific target IP address and port via UDP.
 *
 * @param target_ip The IP address of the recipient.
 * @param target_port The UDP port of the recipient.
 * @param doc The JsonDocument to send.
 * @return True if the packet was successfully handed to the UDP stack, false on error.
 */
bool network_comms_send_json_to_ip_port_udp(IPAddress target_ip, uint16_t target_port, const JsonDocument& doc);

/**
 * @brief Gracefully closes an active TCP client connection from a specific IP address.
 * This is typically called after receiving a "disconnect_notice" JSON message.
 *
 * @param target_ip The IP address of the client to disconnect.
 */
void network_comms_close_client_by_ip(IPAddress target_ip);

/**
 * @brief Gets the local IP address of the ESP32 on the WiFi network.
 * @return IPAddress object. Returns an invalid IP (0.0.0.0) if not connected.
 */
IPAddress network_comms_get_local_ip();


#endif // NETWORK_COMMS_H