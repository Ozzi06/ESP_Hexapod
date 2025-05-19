#ifndef STREAMER_H
#define STREAMER_H

#include "esp_camera.h"
#include "esp_http_server.h"
#include <stdbool.h> // For bool type
#include <stdint.h>  // For uint8_t, uint16_t

// Define common framesize strings for easier mapping
#define FRAMESIZE_STR_QQVGA  "QQVGA"  // 160x120
#define FRAMESIZE_STR_HQVGA  "HQVGA"  // 240x176
#define FRAMESIZE_STR_QVGA   "QVGA"   // 320x240
#define FRAMESIZE_STR_CIF    "CIF"    // 400x296 (Example, add if needed)
#define FRAMESIZE_STR_VGA    "VGA"    // 640x480
// Add more as needed

/**
 * @brief Initializes the camera hardware with specified settings.
 * Call this once at startup.
 *
 * @param initial_framesize The initial resolution for the camera.
 * @param initial_jpeg_quality The initial JPEG quality (0-63, lower is higher quality).
 * @param initial_xclk_mhz The XCLK frequency for the camera in MHz (e.g., 20).
 * @return true if camera initialization was successful, false otherwise.
 */
bool streamer_init_camera(framesize_t initial_framesize, int initial_jpeg_quality, uint8_t initial_xclk_mhz);

/**
 * @brief Sets a camera parameter.
 *
 * @param param_name Name of the parameter (e.g., "framesize", "quality").
 * @param value The integer value for the parameter. For "framesize", this should be a framesize_t enum cast to int.
 * @return true if setting the parameter was successful, false otherwise.
 */
bool streamer_set_camera_param_int(const char* param_name, int value);

/**
 * @brief Sets a camera parameter using a string value, primarily for framesize.
 *
 * @param param_name Name of the parameter (currently only "framesize" uses string value).
 * @param value_str The string value for the parameter (e.g., "QVGA").
 * @return true if setting the parameter was successful, false otherwise.
 */
bool streamer_set_camera_param_str(const char* param_name, const char* value_str);


/**
 * @brief Sets the target frame rate limit for the MJPEG stream.
 * The actual frame rate may be lower due to processing time or camera capabilities.
 * A limit of 0 means no explicit delay-based limiting.
 *
 * @param fps_limit Target frames per second.
 */
void streamer_set_framerate_limit(uint8_t fps_limit);

/**
 * @brief Starts the MJPEG HTTP server on the specified port.
 *
 * @param port The port number for the MJPEG server (e.g., 81).
 * @return true if the server started successfully, false otherwise.
 */
bool streamer_start_mjpeg_server(uint16_t port);

/**
 * @brief Stops the MJPEG HTTP server.
 */
void streamer_stop_mjpeg_server();

/**
 * @brief Checks if the MJPEG stream server is intended to be active.
 * This reflects the desired state, not necessarily if a client is connected.
 *
 * @return true if the stream should be active, false otherwise.
 */
bool streamer_is_active();

/**
 * @brief Gets the current framesize setting as a string.
 * @return Const char pointer to the string representation (e.g., "QVGA"). Returns "UNKNOWN" if not mapped.
 */
const char* streamer_get_current_framesize_str();

/**
 * @brief Gets the current JPEG quality setting.
 * @return Current JPEG quality.
 */
int streamer_get_current_jpeg_quality();

/**
 * @brief Gets the current FPS limit setting.
 * @return Current FPS limit.
 */
uint8_t streamer_get_current_fps_limit();

#endif // STREAMER_H