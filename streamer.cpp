#include "streamer.h"
#include "esp_log.h"
#include "esp_timer.h" // For esp_timer_get_time
#include "sdkconfig.h" // For ESP-IDF configuration like CONFIG_IDF_TARGET_ESP32S3
#include "esp32-hal-psram.h"

// Make sure to include camera_pins.h for your specific board
#include "camera_pins.h" // This should contain the pin definitions for CAMERA_MODEL_XIAO_ESP32S3

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h" // For ESP Arduino core logging macros if used
#define TAG "streamer"
#else
#include "esp_log.h"
static const char *TAG = "streamer";
#endif


// --- HTTP Server and Streaming Configuration ---
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %ld.%06ld\r\n\r\n";

// --- Module Static Variables ---
static httpd_handle_t mjpeg_httpd_server = NULL;
static bool stream_should_be_active = false;

static framesize_t current_framesize_setting = FRAMESIZE_QVGA; // Default
static int current_jpeg_quality_setting = 12;      // Default (0-63, lower is better)
static uint8_t current_framerate_limit_setting = 10; // Default FPS limit
static camera_config_t cam_config; // Global camera configuration

// --- Forward Declarations for Internal Functions ---
static esp_err_t mjpeg_stream_handler_internal(httpd_req_t *req);
static framesize_t map_string_to_framesize(const char* str);
static const char* map_framesize_to_string(framesize_t fs);

// --- Public API Implementations ---

bool streamer_init_camera(framesize_t initial_framesize, int initial_jpeg_quality, uint8_t initial_xclk_mhz) {
    ESP_LOGI(TAG, "Initializing camera...");

    cam_config.ledc_channel = LEDC_CHANNEL_0;
    cam_config.ledc_timer = LEDC_TIMER_0;
    cam_config.pin_d0 = Y2_GPIO_NUM;
    cam_config.pin_d1 = Y3_GPIO_NUM;
    cam_config.pin_d2 = Y4_GPIO_NUM;
    cam_config.pin_d3 = Y5_GPIO_NUM;
    cam_config.pin_d4 = Y6_GPIO_NUM;
    cam_config.pin_d5 = Y7_GPIO_NUM;
    cam_config.pin_d6 = Y8_GPIO_NUM;
    cam_config.pin_d7 = Y9_GPIO_NUM;
    cam_config.pin_xclk = XCLK_GPIO_NUM;
    cam_config.pin_pclk = PCLK_GPIO_NUM;
    cam_config.pin_vsync = VSYNC_GPIO_NUM;
    cam_config.pin_href = HREF_GPIO_NUM;
    cam_config.pin_sccb_sda = SIOD_GPIO_NUM;
    cam_config.pin_sccb_scl = SIOC_GPIO_NUM;
    cam_config.pin_pwdn = PWDN_GPIO_NUM;
    cam_config.pin_reset = RESET_GPIO_NUM;
    cam_config.xclk_freq_hz = initial_xclk_mhz * 1000000;
    cam_config.pixel_format = PIXFORMAT_JPEG;
    cam_config.frame_size = initial_framesize;
    cam_config.jpeg_quality = initial_jpeg_quality;
    cam_config.fb_count = 1; // Minimal frame buffers for simple streaming
    cam_config.grab_mode = CAMERA_GRAB_LATEST; // Discard old frames if busy

    // Adjust for PSRAM
    if (psramFound()) {
        cam_config.fb_location = CAMERA_FB_IN_PSRAM;
        cam_config.fb_count = 2; // Can use more buffers with PSRAM
         ESP_LOGI(TAG, "PSRAM found, using 2 frame buffers in PSRAM.");
    } else {
        cam_config.fb_location = CAMERA_FB_IN_DRAM;
        ESP_LOGW(TAG, "PSRAM not found! Image quality and size may be limited.");
        // Automatically reduce framesize if no PSRAM and trying large format (optional)
        if (initial_framesize > FRAMESIZE_SVGA) {
            ESP_LOGW(TAG, "Large initial framesize without PSRAM, reducing to SVGA.");
            cam_config.frame_size = FRAMESIZE_SVGA;
        }
    }

    esp_err_t err = esp_camera_init(&cam_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return false;
    }
    ESP_LOGI(TAG, "Camera init OK.");

    current_framesize_setting = cam_config.frame_size; // Store the actual used framesize
    current_jpeg_quality_setting = initial_jpeg_quality;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // Apply XIAO ESP32S3 specific sensor settings if any from example (e.g., vflip)
        // Or other generic desirable defaults
        if (s->id.PID == OV3660_PID) { // Example from camera_web_server
            s->set_vflip(s, 1); 
            s->set_brightness(s, 1); 
            s->set_saturation(s, -2);
        }
        #if defined(CAMERA_MODEL_XIAO_ESP32S3) // From example, check if needed for your XIAO camera module
             // s->set_vflip(s, 1); // Apply if your image is upside down
        #endif
        ESP_LOGI(TAG, "Initial sensor settings applied.");
    } else {
        ESP_LOGE(TAG, "Failed to get camera sensor instance.");
        return false;
    }

    return true;
}

static framesize_t map_string_to_framesize(const char* str) {
    if (strcmp(str, FRAMESIZE_STR_QQVGA) == 0) return FRAMESIZE_QQVGA;
    if (strcmp(str, FRAMESIZE_STR_HQVGA) == 0) return FRAMESIZE_HQVGA;
    if (strcmp(str, FRAMESIZE_STR_QVGA) == 0) return FRAMESIZE_QVGA;
    if (strcmp(str, FRAMESIZE_STR_CIF) == 0) return FRAMESIZE_CIF;
    if (strcmp(str, FRAMESIZE_STR_VGA) == 0) return FRAMESIZE_VGA;
    // Add other mappings as defined in streamer.h
    ESP_LOGW(TAG, "Unknown framesize string: %s, defaulting to QVGA", str);
    return FRAMESIZE_QVGA; // Default fallback
}

static const char* map_framesize_to_string(framesize_t fs) {
    switch(fs) {
        case FRAMESIZE_QQVGA: return FRAMESIZE_STR_QQVGA;
        case FRAMESIZE_HQVGA: return FRAMESIZE_STR_HQVGA;
        case FRAMESIZE_QVGA: return FRAMESIZE_STR_QVGA;
        case FRAMESIZE_CIF: return FRAMESIZE_STR_CIF;
        case FRAMESIZE_VGA: return FRAMESIZE_STR_VGA;
        // Add others
        default: return "UNKNOWN";
    }
}

bool streamer_set_camera_param_int(const char* param_name, int value) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "Failed to get sensor for setting param %s", param_name);
        return false;
    }

    int res = -1;
    if (strcmp(param_name, "framesize") == 0) {
        if (s->pixformat == PIXFORMAT_JPEG) { // Can only change framesize if JPEG
            res = s->set_framesize(s, (framesize_t)value);
            if (res == 0) current_framesize_setting = (framesize_t)value;
        } else {
            ESP_LOGW(TAG, "Cannot change framesize, not in JPEG format.");
            res = -1; // Indicate failure
        }
    } else if (strcmp(param_name, "quality") == 0) {
        res = s->set_quality(s, value);
        if (res == 0) current_jpeg_quality_setting = value;
    } else {
        ESP_LOGW(TAG, "Unsupported camera parameter: %s", param_name);
        return false;
    }

    if (res != 0) {
        ESP_LOGE(TAG, "Failed to set %s to %d", param_name, value);
        return false;
    }
    ESP_LOGI(TAG, "Set %s to %d", param_name, value);
    return true;
}

bool streamer_set_camera_param_str(const char* param_name, const char* value_str) {
    if (strcmp(param_name, "framesize") == 0) {
        framesize_t fs = map_string_to_framesize(value_str);
        return streamer_set_camera_param_int("framesize", (int)fs);
    }
    ESP_LOGW(TAG, "Unsupported string parameter or type for: %s", param_name);
    return false;
}


void streamer_set_framerate_limit(uint8_t fps_limit) {
    current_framerate_limit_setting = fps_limit;
    ESP_LOGI(TAG, "MJPEG stream FPS limit set to: %u", fps_limit);
}

bool streamer_start_mjpeg_server(uint16_t port) {
    if (mjpeg_httpd_server) {
        ESP_LOGW(TAG, "MJPEG server already running. Stop it first.");
        return false; // Or stop and restart
    }

    // Apply current camera settings before starting server
    // This is important if settings were changed while server was off
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "Cannot start stream, failed to get sensor.");
        return false;
    }
    if (s->pixformat == PIXFORMAT_JPEG) {
         s->set_framesize(s, current_framesize_setting);
    }
    s->set_quality(s, current_jpeg_quality_setting);


    httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();
    httpd_config.server_port = port;
    httpd_config.ctrl_port = httpd_config.server_port + 10; // Ensure ctrl_port is different, can be arbitrary if not used
    httpd_config.stack_size = 8192; // Increased stack size for httpd task
    httpd_config.max_uri_handlers = 1; // Only one handler for /stream

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = mjpeg_stream_handler_internal,
        .user_ctx = NULL
    };

    ESP_LOGI(TAG, "Starting MJPEG stream server on port: '%d'", httpd_config.server_port);
    if (httpd_start(&mjpeg_httpd_server, &httpd_config) == ESP_OK) {
        if (httpd_register_uri_handler(mjpeg_httpd_server, &stream_uri) == ESP_OK) {
            stream_should_be_active = true;
            ESP_LOGI(TAG, "MJPEG stream server started successfully.");
            return true;
        } else {
            ESP_LOGE(TAG, "Failed to register URI handler for /stream");
            httpd_stop(mjpeg_httpd_server); // Clean up
            mjpeg_httpd_server = NULL;
        }
    } else {
        ESP_LOGE(TAG, "Failed to start MJPEG HTTP server.");
    }
    mjpeg_httpd_server = NULL; // Ensure it's NULL on failure
    return false;
}

void streamer_stop_mjpeg_server() {
    stream_should_be_active = false; // Signal the handler loop to stop

    if (mjpeg_httpd_server) {
        ESP_LOGI(TAG, "Stopping MJPEG stream server...");
        httpd_stop(mjpeg_httpd_server);
        mjpeg_httpd_server = NULL;
        ESP_LOGI(TAG, "MJPEG stream server stopped.");
    } else {
        ESP_LOGI(TAG, "MJPEG stream server was not running.");
    }
}

bool streamer_is_active() {
    return stream_should_be_active && (mjpeg_httpd_server != NULL);
}

const char* streamer_get_current_framesize_str() {
    return map_framesize_to_string(current_framesize_setting);
}

int streamer_get_current_jpeg_quality() {
    return current_jpeg_quality_setting;
}

uint8_t streamer_get_current_fps_limit() {
    return current_framerate_limit_setting;
}


// --- Internal MJPEG Stream Handler ---
static esp_err_t mjpeg_stream_handler_internal(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[128]; // Buffer for multipart headers
    struct timeval _timestamp;

    static int64_t last_frame_sent_time_us = 0;
    int64_t frame_interval_us = 0;


    ESP_LOGI(TAG, "MJPEG client connected. Starting stream...");

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set stream content type.");
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    // httpd_resp_set_hdr(req, "X-Framerate", "None"); // Could update this dynamically if needed

    while (stream_should_be_active) {
        if (current_framerate_limit_setting > 0) {
            frame_interval_us = 1000000LL / current_framerate_limit_setting;
        } else {
            frame_interval_us = 0; // No limit
        }

        int64_t time_now_us = esp_timer_get_time();
        if (frame_interval_us > 0 && (time_now_us - last_frame_sent_time_us < frame_interval_us)) {
            // Simple busy-wait or vTaskDelay for more precise timing if needed for very low FPS
            // For higher FPS, the capture + send time will likely dominate.
            // This basic check helps cap max FPS.
            vTaskDelay(pdMS_TO_TICKS(1)); // Yield for a short moment
            continue;
        }

        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera frame capture failed.");
            // Don't break immediately, client might still be connected, try again
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit before retrying
            continue;
        }

        _timestamp.tv_sec = fb->timestamp.tv_sec;
        _timestamp.tv_usec = fb->timestamp.tv_usec;

        if (fb->format == PIXFORMAT_JPEG) {
            _jpg_buf = fb->buf;
            _jpg_buf_len = fb->len;
        } else {
            // This should not happen if camera is configured for PIXFORMAT_JPEG
            // but as a fallback, one could attempt conversion here.
            // For simplicity, we assume PIXFORMAT_JPEG is already set.
            ESP_LOGW(TAG, "Frame not in JPEG format. Skipping. (Format: %d)", fb->format);
            esp_camera_fb_return(fb);
            fb = NULL;
            continue;
        }

        if (res == ESP_OK) { // Send boundary
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK) { // Send part header
            size_t hlen = snprintf((char *)part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK) { // Send JPEG data
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        esp_camera_fb_return(fb); // Crucial: return frame buffer
        fb = NULL;
        _jpg_buf = NULL; // Buffer was part of fb, not separately allocated here

        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Send frame chunk failed (client likely disconnected). Error: 0x%x", res);
            break; // Exit loop if sending failed
        }
        last_frame_sent_time_us = esp_timer_get_time(); // Update time after successful send
    } // end while(stream_should_be_active)

    ESP_LOGI(TAG, "MJPEG client disconnected or stream stopped.");
    // Finalize response if loop exited due to stream_should_be_active=false but client still connected
    if (res == ESP_OK) {
         httpd_resp_send_chunk(req, NULL, 0); // Send final empty chunk to close connection
    }
    last_frame_sent_time_us = 0; // Reset for next client
    return ESP_OK; // Even if there was a send error, the handler itself completed.
}