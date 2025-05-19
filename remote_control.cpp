// remote_control.cpp
#include <math.h>
#include "remote_control.h"
#include "network_comms.h"
#include "math_utils.h"
#include "servo_angles.h"
#include "battery.h"
#include "robot_spec.h"
#include "walkcycle.h"
#include "streamer.h"

// --- Configuration & Global State for Remote Control ---
static bool rc_log_network_packets = true;
static unsigned long rc_last_packet_received_ms = 0;

// Discovery Beacon
const uint16_t DISCOVERY_BROADCAST_PORT = 5008;
const unsigned long DISCOVERY_BEACON_INTERVAL_MS = 2000;
static unsigned long last_discovery_beacon_sent_ms = 0;

// --- ESP32 Owned Configuration Parameters ---
static float cfg_max_linear_speed_cms = 8.0f;
static float cfg_max_yaw_rate_rads = 0.3f;
static float cfg_pose_adjust_linear_cms = 2.0f;
static float cfg_pose_adjust_angular_rads = 0.26f; 
static float cfg_linear_acceleration_cm_s2 = 10.0f;
static float cfg_linear_deceleration_cm_s2 = 20.0f;
static float cfg_yaw_acceleration_rad_s2 = 30.0f * M_PI / 180.0f;
static float cfg_yaw_deceleration_rad_s2 = 60.0f * M_PI / 180.0f; 

static float cfg_leg_geom_front_corner_x_cm = 20.0f;
static float cfg_leg_geom_front_corner_y_cm = 23.0f;
static float cfg_leg_geom_middle_side_x_cm  = 27.0f;
static float cfg_leg_geom_corner_ext_cm     = 0.0f;
static float cfg_leg_geom_middle_ext_cm     = 0.0f;

// --- ESP32 Owned Intent/Active State Variables ---
static float active_target_vx_factor = 0.0f;
static float active_target_vy_factor = 0.0f;
static float active_target_yaw_factor = 0.0f;
static float active_offset_x_direction = 0.0f;
static float active_offset_y_direction = 0.0f;
static float active_offset_z_direction = 0.0f;
static float active_pitch_direction = 0.0f;
static float active_roll_direction = 0.0f;
static float active_body_yaw_direction = 0.0f;
static bool active_centering_xy = false;
static bool active_centering_orientation = false;

// --- Mobile App Specific Input State ---
static bool app_is_enabled_by_switch = false;
static float app_raw_joystick_x = 0.0f;
static float app_raw_joystick_y = 0.0f;
static float app_raw_steering_angle_deg = 0.0f;
static bool app_hold_yaw_plus = false;
static bool app_hold_yaw_minus = false;
static bool app_hold_dpad_xy_up = false, app_hold_dpad_xy_down = false, app_hold_dpad_xy_left = false, app_hold_dpad_xy_right = false;
static bool app_hold_dpad_rot_pitch_up = false, app_hold_dpad_rot_pitch_down = false, app_hold_dpad_rot_roll_left = false, app_hold_dpad_rot_roll_right = false;
static bool app_hold_center_xy = false;
static bool app_hold_center_rot = false;
static bool app_hold_body_z_up = false;
static bool app_hold_body_z_down = false;

// --- Global Telemetry State (Single Target/GUI) ---
static IPAddress global_telemetry_destination_ip;
static uint16_t global_telemetry_destination_udp_port = 0;
static bool global_telemetry_target_known = false;

struct TelemetrySubscription {
  bool enabled = false;
  uint32_t interval_ms = 1000;
  uint32_t last_sent_ms = 0;
};
static TelemetrySubscription sub_battery;
static TelemetrySubscription sub_robot_status;
static TelemetrySubscription sub_robot_state_actual;
static TelemetrySubscription sub_debug_foot_pos;


// --- Forward Declarations for internal functions ---
static void process_json_packet_internal(const JsonDocument& doc, IPAddress source_ip, uint16_t source_port, bool is_tcp, WiFiClient tcp_client); // Added source_port
static void processLocomotionIntent(JsonObjectConst payload);
static void processPoseAdjustIntent(JsonObjectConst payload);
static void processCenteringIntent(JsonObjectConst payload);
static void processGaitCommand(JsonObjectConst payload);
static void processConfigUpdate(JsonObjectConst payload);
static void processClientSettings(JsonObjectConst payload, IPAddress source_ip);
static void processRequestFullState(IPAddress reply_to_ip);
static void processDisconnectNotice(JsonObjectConst payload);
static void processSystemCommand(JsonObjectConst payload);
static void processPingCommand(JsonObjectConst payload, IPAddress source_ip, WiFiClient tcp_client);
static void processCameraStreamControl(JsonObjectConst payload, IPAddress source_ip);
static void processCameraConfigUpdate(JsonObjectConst payload, IPAddress source_ip);

static void parseAndStoreMobileAppInput(const JsonDocument& doc);
static void translateMobileInputsToActiveIntents();
static void integrateRobotState(float dt);
static void sendConfiguredTelemetry();
static void sendDiscoveryBeacon(); // New for Discovery
static void clear_all_subscriptions();
static void calculate_and_update_base_foot_positions_from_abstract_config();


// --- Callbacks for network_comms ---
static void json_packet_received_callback(const JsonDocument& doc, IPAddress source_ip, uint16_t source_port, bool is_tcp, WiFiClient tcp_client) {
    rc_last_packet_received_ms = millis(); 

    if (rc_log_network_packets) {
        Serial.printf("[%s RX %s:%u] ", is_tcp ? "TCP" : "UDP", source_ip.toString().c_str(), source_port);
        // serializeJsonPretty(doc, Serial); // Raw JSON might be logged by network_comms
        // Serial.println();
    }
    process_json_packet_internal(doc, source_ip, source_port, is_tcp, tcp_client);
}

static void tcp_client_abrupt_disconnect_callback(IPAddress disconnected_client_ip) {
    if (rc_log_network_packets) {
        Serial.printf("[RC] TCP client %s abruptly disconnected.\n", disconnected_client_ip.toString().c_str());
    }
    if (global_telemetry_target_known && global_telemetry_destination_ip == disconnected_client_ip) {
        Serial.println("[RC] Disconnected client was the telemetry target. Clearing subscriptions.");
        clear_all_subscriptions();
        global_telemetry_target_known = false;
        global_telemetry_destination_ip = IPAddress(); 
    }
}

// --- Public Functions ---

void setupRemoteControl() {
  Serial.println("--- Remote Control Mode Setup ---");
  calculate_and_update_base_foot_positions_from_abstract_config();
  setupWalkcycle();

  framesize_t initial_fs = FRAMESIZE_QVGA; // Your desired default
  int initial_jq = 12;                   // Your desired default
  uint8_t initial_xclk = 30;             // Default from example
  if (!streamer_init_camera(initial_fs, initial_jq, initial_xclk)) {
      Serial.println("[ERROR] Failed to initialize camera via streamer module!");
      // Handle error, maybe blink LED, etc.
  } else {
      // Store initial settings in your global config vars if needed for full_state_response
      // For example:
      // cfg_camera_framesize_str = streamer_get_current_framesize_str();
      // cfg_camera_quality = streamer_get_current_jpeg_quality();
      // cfg_camera_fps_limit = streamer_get_current_fps_limit(); // (initial_fps_limit should be set via streamer_set_framerate_limit)
      streamer_set_framerate_limit(10); // Example default FPS limit
  }

  rc_log_network_packets = false; 
  rc_last_packet_received_ms = millis();
  last_discovery_beacon_sent_ms = millis(); // Initialize beacon timer

  cfg_max_linear_speed_cms = 8.0f;
  cfg_max_yaw_rate_rads = 0.3f;
  cfg_pose_adjust_linear_cms = 5.0f;
  cfg_pose_adjust_angular_rads = 15.0f * M_PI / 180.0f;

  sub_battery.enabled = false; sub_battery.interval_ms = 1000;
  sub_robot_status.enabled = false; sub_robot_status.interval_ms = 5000;
  sub_robot_state_actual.enabled = false; sub_robot_state_actual.interval_ms = 200;
  sub_debug_foot_pos.enabled = false; sub_debug_foot_pos.interval_ms = 200;
  global_telemetry_target_known = false;

  if (WiFi.status() == WL_CONNECTED) {
    if (!network_comms_setup(TCP_LISTEN_PORT, UDP_LISTEN_PORT, json_packet_received_callback, tcp_client_abrupt_disconnect_callback)) {
      Serial.println("[RC ERROR] Failed to setup network_comms!");
    } else {
      Serial.println("[RC] network_comms initialized.");
      Serial.print("[RC] ESP32 Local IP: "); Serial.println(network_comms_get_local_ip().toString());
    }
  } else {
    Serial.println("[RC WARN] WiFi not connected. Network communications not started.");
  }
  printRemoteControlSerialHelp();
}

bool remoteControlUpdate() {
  unsigned long currentTimeMs = millis();
  static unsigned long lastLoopMicros = 0;
  if (lastLoopMicros == 0) lastLoopMicros = micros();
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = currentMicros;
  if (dt <= 0.0f || dt > 0.1f) { dt = 0.02f; }

  network_comms_handle();

  if (Serial.available() > 0) {
    char command = toupper(Serial.read());
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) { Serial.read(); } 
    Serial.print("RemoteMode CMD> "); Serial.println(command);
    switch (command) {
      case 'L': rc_log_network_packets = !rc_log_network_packets; Serial.printf("RC Network Packet Logging: %s\n", rc_log_network_packets ? "ON" : "OFF"); break;
      case 'X': walkCycleRunning = false; bodyVelocity = {0,0,0}; bodyAngularVelocityYaw = 0; return false;
      case 'H': case '?': printRemoteControlSerialHelp(); break;
      default: Serial.println("Unknown RemoteMode CMD."); break;
    }
  }

  if (millis() - rc_last_packet_received_ms > GLOBAL_PACKET_TIMEOUT_MS) { //using millis again to ensure rc_last_packet_received_ms is not at a later timestamp so it becomes negative
    Serial.println("[RC WARN] GLOBAL PACKET TIMEOUT! Halting motion.");
    active_target_vx_factor = 0.0f; active_target_vy_factor = 0.0f; active_target_yaw_factor = 0.0f;
    active_offset_x_direction = 0.0f; active_offset_y_direction = 0.0f; active_offset_z_direction = 0.0f;
    active_pitch_direction = 0.0f; active_roll_direction = 0.0f; active_body_yaw_direction = 0.0f;
    active_centering_xy = false; active_centering_orientation = false;
    walkCycleRunning = false;
    rc_last_packet_received_ms = millis(); 
  }

  if (app_is_enabled_by_switch) {
    translateMobileInputsToActiveIntents();
  }

  integrateRobotState(dt);

  if (walkCycleRunning) {
    updateWalkCycle(dt);
  }

  if (global_telemetry_target_known) {
    sendConfiguredTelemetry();
  }

  // Send discovery beacon periodically
  if (WiFi.status() == WL_CONNECTED && (currentTimeMs - last_discovery_beacon_sent_ms >= DISCOVERY_BEACON_INTERVAL_MS)) {
      sendDiscoveryBeacon();
      last_discovery_beacon_sent_ms = currentTimeMs;
  }

  return true; 
}

void printRemoteControlSerialHelp() {
  Serial.println("\n--- Remote Control Mode Serial ---");
  Serial.println("  L - Toggle RC Network Packet Logging");
  Serial.println("  X - Exit to Main Menu");
  Serial.println("  H / ? - Display this help");
  Serial.println("--------------------------------");
}


// --- Internal JSON Packet Processing Logic ---
static void process_json_packet_internal(const JsonDocument& doc, IPAddress source_ip, uint16_t source_port, bool is_tcp, WiFiClient tcp_client) {
  JsonVariantConst type_variant = doc["type"];
  if (!type_variant.is<const char*>()) {
    if (rc_log_network_packets) Serial.println("[RC] JSON no 'type' string.");
    return;
  }
  const char* type = type_variant.as<const char*>();
  JsonVariantConst payload_variant = doc["payload"];
  
  // --- Mobile App Specific Input Handling ---
  if (doc.containsKey("id") &&
      (strcmp(type, "JOYSTICK") == 0 || strcmp(type, "BUTTON") == 0 ||
       strcmp(type, "DPAD") == 0 || strcmp(type, "SWITCH") == 0 ||
       strcmp(type, "STEERING_WHEEL") == 0 || strcmp(type, "SLIDER") == 0)) {
    parseAndStoreMobileAppInput(doc);
    return;
  }
  
  // --- Standard GUI/Remote Command Handling ---
  if (strcmp(type, "locomotion_intent") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processLocomotionIntent(payload_variant.as<JsonObjectConst>());
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "pose_adjust_intent") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processPoseAdjustIntent(payload_variant.as<JsonObjectConst>());
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "centering_intent") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processCenteringIntent(payload_variant.as<JsonObjectConst>());
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "gait_command") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processGaitCommand(payload_variant.as<JsonObjectConst>());
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "config_update") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processConfigUpdate(payload_variant.as<JsonObjectConst>());
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "client_settings") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processClientSettings(payload_variant.as<JsonObjectConst>(), source_ip);
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "system_command") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processSystemCommand(payload_variant.as<JsonObjectConst>());
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "request_full_state") == 0) {
      JsonVariantConst reply_ip_var = doc["reply_to_ip"]; 
      if (reply_ip_var.is<const char*>()) {
          IPAddress reply_ip;
          if (reply_ip.fromString(reply_ip_var.as<const char*>())) {
              processRequestFullState(reply_ip);
          } else if (rc_log_network_packets) Serial.println("[RC] Invalid reply_to_ip in request_full_state.");
      } else if (rc_log_network_packets) Serial.println("[RC] Missing reply_to_ip in request_full_state.");
  } else if (strcmp(type, "disconnect_notice") == 0) {
    if (payload_variant.is<JsonObjectConst>()) processDisconnectNotice(payload_variant.as<JsonObjectConst>());
    else if (rc_log_network_packets) Serial.printf("[RC] Missing payload for %s\n", type);
  } else if (strcmp(type, "ping") == 0) {
    if (is_tcp && payload_variant.is<JsonObjectConst>()) {
        processPingCommand(payload_variant.as<JsonObjectConst>(), source_ip, tcp_client);
    } else if (rc_log_network_packets) Serial.println("[RC] Invalid or non-TCP ping received.");
  }
  // --- New Camera Control Message Types ---
  else if (strcmp(type, "camera_stream_control") == 0) {
    if (is_tcp && payload_variant.is<JsonObjectConst>()) { // Camera control should be reliable TCP
        processCameraStreamControl(payload_variant.as<JsonObjectConst>(), source_ip);
    } else if (rc_log_network_packets) Serial.println("[RC] Invalid or non-TCP camera_stream_control received.");
  } else if (strcmp(type, "camera_config_update") == 0) {
    if (is_tcp && payload_variant.is<JsonObjectConst>()) { // Camera config should be reliable TCP
        processCameraConfigUpdate(payload_variant.as<JsonObjectConst>(), source_ip);
    } else if (rc_log_network_packets) Serial.println("[RC] Invalid or non-TCP camera_config_update received.");
  }
  // --- End of New Camera Control ---
  else {
    if (rc_log_network_packets) { Serial.print("[RC] Unknown JSON msg type: "); Serial.println(type); }
  }
}

static void processLocomotionIntent(JsonObjectConst payload) {
  if (app_is_enabled_by_switch) return;
  active_target_vx_factor = payload["target_vx_factor"] | 0.0f;
  active_target_vy_factor = payload["target_vy_factor"] | 0.0f;
  active_target_yaw_factor = payload["target_yaw_factor"] | 0.0f;
}

static void processPoseAdjustIntent(JsonObjectConst payload) {
  if (app_is_enabled_by_switch) return;
  active_offset_x_direction = payload["offset_x_active"] | 0.0f;
  active_offset_y_direction = payload["offset_y_active"] | 0.0f;
  active_offset_z_direction = payload["offset_z_active"] | 0.0f;
  active_pitch_direction = payload["pitch_active"] | 0.0f;
  active_roll_direction = payload["roll_active"] | 0.0f;
  active_body_yaw_direction = payload["body_yaw_active"] | 0.0f;
}

static void processCenteringIntent(JsonObjectConst payload) {
  if (app_is_enabled_by_switch) return;
  active_centering_xy = payload["center_xy_active"] | false;
  active_centering_orientation = payload["center_orientation_active"] | false;
}

static void processGaitCommand(JsonObjectConst payload) {
  if (!app_is_enabled_by_switch) { // Only allow GUI to control gait if app switch is OFF
    if (payload.containsKey("walk_active")) {
        walkCycleRunning = payload["walk_active"].as<bool>();
        if (rc_log_network_packets) Serial.printf("[RC] GUI set Walk Active to: %s\n", walkCycleRunning ? "true" : "false");
    }
  } else {
      if (rc_log_network_packets) Serial.println("[RC] Gait command from GUI ignored, Mobile App switch is ON.");
  }
}

static void processConfigUpdate(JsonObjectConst payload) {
  if (rc_log_network_packets) Serial.println("[RC] Processing 'config_update'");
  if (payload.containsKey("max_speeds")) {
    JsonObjectConst speeds = payload["max_speeds"].as<JsonObjectConst>();
    cfg_max_linear_speed_cms = speeds["linear_cms"] | cfg_max_linear_speed_cms;
    cfg_max_yaw_rate_rads = speeds["yaw_rads"] | cfg_max_yaw_rate_rads;
    if (rc_log_network_packets) Serial.printf("  Max speeds updated: Lin=%.1f cm/s, Yaw=%.2f rad/s\n", cfg_max_linear_speed_cms, cfg_max_yaw_rate_rads);
  }
  if (payload.containsKey("pose_adjust_speeds")) {
    JsonObjectConst speeds = payload["pose_adjust_speeds"].as<JsonObjectConst>();
    cfg_pose_adjust_linear_cms = speeds["linear_cms"] | cfg_pose_adjust_linear_cms;
    cfg_pose_adjust_angular_rads = speeds["angular_rads"] | cfg_pose_adjust_angular_rads;
     if (rc_log_network_packets) Serial.printf("  Pose speeds updated: Lin=%.1f cm/s, Ang=%.2f rad/s\n", cfg_pose_adjust_linear_cms, cfg_pose_adjust_angular_rads);
  }
  if (payload.containsKey("gait_params")) {
    JsonObjectConst gait = payload["gait_params"].as<JsonObjectConst>();
    walkParams.stepHeight = gait["step_height_cm"] | walkParams.stepHeight;
    walkParams.stepTime = gait["step_time_s"] | walkParams.stepTime;
    if (rc_log_network_packets) Serial.printf("  Gait params updated: Height=%.1f cm, Time=%.1f s\n", walkParams.stepHeight, walkParams.stepTime);
  }
  if (payload.containsKey("leg_geometry_abstract")) {
     JsonObjectConst geom_abstract = payload["leg_geometry_abstract"].as<JsonObjectConst>();
     cfg_leg_geom_front_corner_x_cm = geom_abstract["front_corner_x_cm"] | cfg_leg_geom_front_corner_x_cm;
     cfg_leg_geom_front_corner_y_cm = geom_abstract["front_corner_y_cm"] | cfg_leg_geom_front_corner_y_cm;
     cfg_leg_geom_middle_side_x_cm  = geom_abstract["middle_side_x_cm"]  | cfg_leg_geom_middle_side_x_cm;
     cfg_leg_geom_corner_ext_cm     = geom_abstract["corner_ext_cm"]     | cfg_leg_geom_corner_ext_cm;
     cfg_leg_geom_middle_ext_cm     = geom_abstract["middle_ext_cm"]     | cfg_leg_geom_middle_ext_cm;
     
     calculate_and_update_base_foot_positions_from_abstract_config();

     if (rc_log_network_packets) {
         Serial.println("  Abstract leg geometry updated by GUI. Base foot positions recalculated.");
     }
  }
  if (payload.containsKey("acceleration_values")) {
     JsonObjectConst accel_vals = payload["acceleration_values"].as<JsonObjectConst>();
     cfg_linear_acceleration_cm_s2 = accel_vals["linear_accel_cms2"] | cfg_linear_acceleration_cm_s2;
     cfg_linear_deceleration_cm_s2 = accel_vals["linear_decel_cms2"] | cfg_linear_deceleration_cm_s2;
     cfg_yaw_acceleration_rad_s2 = accel_vals["yaw_accel_rads2"] | cfg_yaw_acceleration_rad_s2;
     cfg_yaw_deceleration_rad_s2 = accel_vals["yaw_decel_rads2"] | cfg_yaw_deceleration_rad_s2;
     if (rc_log_network_packets) {
         Serial.printf("  Accel values updated: LinAccel=%.1f, LinDecel=%.1f, YawAccel=%.2f, YawDecel=%.2f\n",
                       cfg_linear_acceleration_cm_s2, cfg_linear_deceleration_cm_s2,
                       cfg_yaw_acceleration_rad_s2, cfg_yaw_deceleration_rad_s2);
     }
  }
}

static void processClientSettings(JsonObjectConst payload, IPAddress source_ip_of_settings_sender) {
    if (rc_log_network_packets) Serial.printf("[RC] Processing 'client_settings' from %s\n", source_ip_of_settings_sender.toString().c_str());

    JsonObjectConst udp_config = payload["udp_telemetry_config"].as<JsonObjectConst>();
    JsonObjectConst tcp_subs_config = payload["tcp_subscriptions_here"].as<JsonObjectConst>();

    if (udp_config) {
        JsonVariantConst ip_var = udp_config["target_ip"];
        JsonVariantConst port_var = udp_config["target_port"];
        IPAddress new_dest_ip;
        bool ip_ok = false;
        if (ip_var.is<const char*>()) {
            ip_ok = new_dest_ip.fromString(ip_var.as<const char*>());
        }

        if (ip_ok && port_var.is<unsigned int>()) {
            global_telemetry_destination_ip = new_dest_ip;
            global_telemetry_destination_udp_port = port_var.as<unsigned int>();
            global_telemetry_target_known = true; 
            if (rc_log_network_packets) {
                Serial.printf("  UDP Telemetry target set to: %s:%u\n",
                              global_telemetry_destination_ip.toString().c_str(), global_telemetry_destination_udp_port);
            }
        } else {
            if (rc_log_network_packets) Serial.println("  [WARN] Invalid target_ip or target_port in udp_telemetry_config.");
        }
        
        JsonObjectConst udp_subs = udp_config["subscriptions"].as<JsonObjectConst>();
        if (udp_subs) {
            if (rc_log_network_packets) Serial.println("  Updating UDP subscriptions:");
            if (udp_subs["robot_state_actual"].is<JsonObjectConst>()) {
                sub_robot_state_actual.enabled = udp_subs["robot_state_actual"]["enabled"] | false;
                sub_robot_state_actual.interval_ms = udp_subs["robot_state_actual"]["interval_ms"] | 200;
                sub_robot_state_actual.last_sent_ms = 0; 
                if (rc_log_network_packets) Serial.printf("    RobotStateActual (UDP): %s, %ums\n", sub_robot_state_actual.enabled?"ON":"OFF", sub_robot_state_actual.interval_ms);
            }
            if (udp_subs["debug_foot_pos"].is<JsonObjectConst>()) {
                sub_debug_foot_pos.enabled = udp_subs["debug_foot_pos"]["enabled"] | false;
                sub_debug_foot_pos.interval_ms = udp_subs["debug_foot_pos"]["interval_ms"] | 200;
                sub_debug_foot_pos.last_sent_ms = 0;
                if (rc_log_network_packets) Serial.printf("    DebugFootPos (UDP): %s, %ums\n", sub_debug_foot_pos.enabled?"ON":"OFF", sub_debug_foot_pos.interval_ms);
            }
        } else if (rc_log_network_packets) Serial.println("  No 'subscriptions' field in udp_telemetry_config.");
    } else if (rc_log_network_packets) Serial.println("  No 'udp_telemetry_config' field in client_settings.");


    if (tcp_subs_config) {
        if (rc_log_network_packets) Serial.println("  Updating TCP subscriptions:");
        if (tcp_subs_config["battery"].is<JsonObjectConst>()) {
            sub_battery.enabled = tcp_subs_config["battery"]["enabled"] | false;
            sub_battery.interval_ms = tcp_subs_config["battery"]["interval_ms"] | 1000;
            sub_battery.last_sent_ms = 0;
            if (rc_log_network_packets) Serial.printf("    Battery (TCP): %s, %ums\n", sub_battery.enabled?"ON":"OFF", sub_battery.interval_ms);
        }
        if (tcp_subs_config["robot_status"].is<JsonObjectConst>()) {
            sub_robot_status.enabled = tcp_subs_config["robot_status"]["enabled"] | false;
            sub_robot_status.interval_ms = tcp_subs_config["robot_status"]["interval_ms"] | 5000;
            sub_robot_status.last_sent_ms = 0;
            if (rc_log_network_packets) Serial.printf("    RobotStatus (TCP): %s, %ums\n", sub_robot_status.enabled?"ON":"OFF", sub_robot_status.interval_ms);
        }
    } else if (rc_log_network_packets) Serial.println("  No 'tcp_subscriptions_here' field in client_settings.");
}

static void processRequestFullState(IPAddress reply_to_ip) {
    if (rc_log_network_packets) Serial.printf("[RC] Processing 'request_full_state' for IP %s\n", reply_to_ip.toString().c_str());
    if (!reply_to_ip || reply_to_ip == INADDR_NONE) {
        if (rc_log_network_packets) Serial.println("  [WARN] Invalid reply_to_ip for request_full_state.");
        return;
    }

    // Increased size slightly for camera config, adjust if more state is added.
    DynamicJsonDocument full_state_doc(1280); // Adjusted size
    full_state_doc["type"] = "full_state_response";
    full_state_doc["source"] = "esp32_hexapod";
    
    JsonObject payload = full_state_doc.createNestedObject("payload");

    // --- Existing state items ---
    JsonObject speeds = payload.createNestedObject("max_speeds");
    speeds["linear_cms"] = cfg_max_linear_speed_cms;
    speeds["yaw_rads"] = cfg_max_yaw_rate_rads;

    JsonObject pose_speeds = payload.createNestedObject("pose_adjust_speeds");
    pose_speeds["linear_cms"] = cfg_pose_adjust_linear_cms;
    pose_speeds["angular_rads"] = cfg_pose_adjust_angular_rads;

    JsonObject accel_vals = payload.createNestedObject("acceleration_values");
    accel_vals["linear_accel_cms2"] = cfg_linear_acceleration_cm_s2;
    accel_vals["linear_decel_cms2"] = cfg_linear_deceleration_cm_s2;
    accel_vals["yaw_accel_rads2"] = cfg_yaw_acceleration_rad_s2;
    accel_vals["yaw_decel_rads2"] = cfg_yaw_deceleration_rad_s2;

    JsonObject gait = payload.createNestedObject("gait_params");
    gait["step_height_cm"] = walkParams.stepHeight;
    gait["step_time_s"] = walkParams.stepTime;
    
    payload["walk_active"] = walkCycleRunning; 
    
    JsonObject geom_abstract_payload = payload.createNestedObject("leg_geometry_abstract");
    geom_abstract_payload["front_corner_x_cm"] = cfg_leg_geom_front_corner_x_cm;
    geom_abstract_payload["front_corner_y_cm"] = cfg_leg_geom_front_corner_y_cm;
    geom_abstract_payload["middle_side_x_cm"]  = cfg_leg_geom_middle_side_x_cm;
    geom_abstract_payload["corner_ext_cm"]     = cfg_leg_geom_corner_ext_cm;
    geom_abstract_payload["middle_ext_cm"]     = cfg_leg_geom_middle_ext_cm;

    JsonArray base_pos_array = payload.createNestedArray("base_foot_positions_walk_cm");
    for(int i=0; i<LEG_COUNT; ++i) {
        JsonObject leg_obj = base_pos_array.createNestedObject();
        leg_obj["x"] = baseFootPositionWalk[i].x;
        leg_obj["y"] = baseFootPositionWalk[i].y;
        leg_obj["z"] = baseFootPositionWalk[i].z;
    }

    JsonObject current_pose = payload.createNestedObject("current_body_pose");
    JsonObject pos_offset = current_pose.createNestedObject("position_offset_cm");
    pos_offset["x"] = bodyPositionOffset.x;
    pos_offset["y"] = bodyPositionOffset.y;
    pos_offset["z"] = bodyPositionOffset.z;
    JsonObject orient_q = current_pose.createNestedObject("orientation_quat");
    orient_q["w"] = bodyOrientation.w;
    orient_q["x"] = bodyOrientation.x;
    orient_q["y"] = bodyOrientation.y;
    orient_q["z"] = bodyOrientation.z;

    // --- New Camera Configuration State ---
    JsonObject camera_cfg = payload.createNestedObject("camera_config");
    camera_cfg["resolution"] = streamer_get_current_framesize_str();
    camera_cfg["quality"] = streamer_get_current_jpeg_quality();
    camera_cfg["fps_limit"] = streamer_get_current_fps_limit();
    camera_cfg["stream_active"] = streamer_is_active(); // Also report if stream is currently supposed to be active

    // --- Send the complete state ---
    if (!network_comms_send_json_to_ip_tcp(reply_to_ip, full_state_doc)) {
        if (rc_log_network_packets) Serial.printf("  [WARN] Failed to send full_state_response to %s\n", reply_to_ip.toString().c_str());
    } else if (rc_log_network_packets) Serial.printf("  Sent full_state_response to %s\n", reply_to_ip.toString().c_str());
}

static void processDisconnectNotice(JsonObjectConst payload) {
    JsonVariantConst ip_var = payload["source_ip"];
    if (ip_var.is<const char*>()) {
        IPAddress source_ip_of_disconnecting_client;
        if (source_ip_of_disconnecting_client.fromString(ip_var.as<const char*>())) {
            if (rc_log_network_packets) Serial.printf("[RC] Processing 'disconnect_notice' from %s\n", source_ip_of_disconnecting_client.toString().c_str());
            if (global_telemetry_target_known && global_telemetry_destination_ip == source_ip_of_disconnecting_client) {
                Serial.println("  Client was telemetry target. Clearing subscriptions.");
                clear_all_subscriptions();
                global_telemetry_target_known = false;
                global_telemetry_destination_ip = IPAddress();
            }
            network_comms_close_client_by_ip(source_ip_of_disconnecting_client);
        } else if (rc_log_network_packets) Serial.println("  [WARN] Invalid source_ip in disconnect_notice.");
    } else if (rc_log_network_packets) Serial.println("  [WARN] Missing source_ip in disconnect_notice.");
}

static void processSystemCommand(JsonObjectConst payload) {
  if (rc_log_network_packets) Serial.println("[RC] Processing 'system_command'");
  const char* action = payload["action"];
  if (action && strcmp(action, "reinitialize_pwm") == 0) {
    Serial.println("  CMD: Re-initializing PWM drivers...");
    setupPwm();
    Serial.println("  PWM drivers re-initialized.");
  }
}

static void processPingCommand(JsonObjectConst payload, IPAddress source_ip, WiFiClient tcp_client) {
    if (!tcp_client || !tcp_client.connected()) {
        if (rc_log_network_packets) Serial.printf("[RC Ping] No valid TCP client to send pong to %s.\n", source_ip.toString().c_str());
        return;
    }

    JsonVariantConst gui_ts_var = payload["ts"];
    double gui_ts = gui_ts_var.as<double>(); // GUI sends float/double from time.time()

    StaticJsonDocument<256> pong_doc;
    pong_doc["type"] = "pong";
    pong_doc["source"] = "esp32_hexapod";
    JsonObject pong_payload = pong_doc.createNestedObject("payload");
    pong_payload["original_ts"] = gui_ts; // Echo back the GUI's timestamp
    pong_payload["esp_reply_ts"] = millis();

    if (network_comms_send_json_to_ip_tcp(source_ip, pong_doc)) {
        if (rc_log_network_packets) Serial.printf("[RC Ping] Sent pong to %s.\n", source_ip.toString().c_str());
    } else {
        if (rc_log_network_packets) Serial.printf("[RC Ping WARN] Failed to send pong to %s.\n", source_ip.toString().c_str());
    }
}

static void processCameraStreamControl(JsonObjectConst payload, IPAddress source_ip) {
    const char* action = payload["action"];
    bool success = false;
    const char* status_message = "Unknown action";

    if (rc_log_network_packets) {
        Serial.printf("[RC] Processing 'camera_stream_control' from %s. Action: %s\n",
                      source_ip.toString().c_str(), action ? action : "NULL");
    }

    if (action) {
        if (strcmp(action, "start") == 0) {
            if (streamer_start_mjpeg_server(81)) { // Port 81 hardcoded as discussed
                success = true;
                status_message = "Stream server started.";
                if (rc_log_network_packets) Serial.println("  Streamer Start OK.");
            } else {
                status_message = "Failed to start stream server.";
                if (rc_log_network_packets) Serial.println("  Streamer Start FAILED.");
            }
        } else if (strcmp(action, "stop") == 0) {
            streamer_stop_mjpeg_server();
            success = true; // Stopping is generally always "successful" from a command perspective
            status_message = "Stream server stopped.";
            if (rc_log_network_packets) Serial.println("  Streamer Stop executed.");
        } else {
            if (rc_log_network_packets) Serial.printf("  Unknown camera_stream_control action: %s\n", action);
        }
    } else {
        status_message = "No action specified.";
        if (rc_log_network_packets) Serial.println("  'camera_stream_control' missing 'action' field.");
    }

    // Send Acknowledgement
    StaticJsonDocument<256> ack_doc;
    ack_doc["type"] = "camera_stream_ack";
    ack_doc["source"] = "esp32_hexapod";
    JsonObject ack_payload = ack_doc.createNestedObject("payload");
    ack_payload["action_requested"] = action ? action : "unknown";
    ack_payload["success"] = success;
    ack_payload["message"] = status_message;

    network_comms_send_json_to_ip_tcp(source_ip, ack_doc);
}

static void processCameraConfigUpdate(JsonObjectConst payload, IPAddress source_ip) {
    if (rc_log_network_packets) {
        Serial.printf("[RC] Processing 'camera_config_update' from %s\n", source_ip.toString().c_str());
    }

    bool config_changed = false;
    bool overall_success = true; // Assume success unless a set operation fails

    // Stop stream if active, apply settings, then restart if it was active
    // This is "Option A" for simplicity.
    bool stream_was_active = streamer_is_active();
    if (stream_was_active) {
        if (rc_log_network_packets) Serial.println("  Stream was active, stopping to apply new config...");
        streamer_stop_mjpeg_server();
    }

    if (payload.containsKey("resolution")) {
        const char* res_str = payload["resolution"];
        if (res_str) {
            if (streamer_set_camera_param_str("framesize", res_str)) {
                if (rc_log_network_packets) Serial.printf("  Set camera resolution to: %s\n", res_str);
                config_changed = true;
            } else {
                if (rc_log_network_packets) Serial.printf("  Failed to set camera resolution: %s\n", res_str);
                overall_success = false;
            }
        }
    }

    if (payload.containsKey("quality")) {
        int quality = payload["quality"] | streamer_get_current_jpeg_quality(); // Default to current if not specified
        if (streamer_set_camera_param_int("quality", quality)) {
            if (rc_log_network_packets) Serial.printf("  Set camera quality to: %d\n", quality);
            config_changed = true;
        } else {
            if (rc_log_network_packets) Serial.printf("  Failed to set camera quality: %d\n", quality);
            overall_success = false;
        }
    }

    if (payload.containsKey("fps_limit")) {
        uint8_t fps = payload["fps_limit"] | streamer_get_current_fps_limit(); // Default to current
        streamer_set_framerate_limit(fps); // This function doesn't return bool, assume success
        if (rc_log_network_packets) Serial.printf("  Set camera FPS limit to: %u\n", fps);
        config_changed = true; // Even if value is same, considered a config "touch"
    }

    if (stream_was_active) {
        if (rc_log_network_packets) Serial.println("  Restarting stream with new config...");
        if (!streamer_start_mjpeg_server(81)) {
             if (rc_log_network_packets) Serial.println("  Failed to restart stream server after config update!");
             overall_success = false; // If it was active, it should be able to restart
        } else {
            if (rc_log_network_packets) Serial.println("  Stream restarted successfully.");
        }
    }

    // Send Acknowledgement
    StaticJsonDocument<256> ack_doc;
    ack_doc["type"] = "camera_config_ack";
    ack_doc["source"] = "esp32_hexapod";
    JsonObject ack_payload = ack_doc.createNestedObject("payload");
    ack_payload["success"] = overall_success;
    ack_payload["message"] = overall_success ? "Config updated." : "Config update partially or fully failed.";
    // Include current settings in ACK for confirmation
    ack_payload["current_resolution"] = streamer_get_current_framesize_str();
    ack_payload["current_quality"] = streamer_get_current_jpeg_quality();
    ack_payload["current_fps_limit"] = streamer_get_current_fps_limit();

    network_comms_send_json_to_ip_tcp(source_ip, ack_doc);
}


static void clear_all_subscriptions() {
    sub_battery.enabled = false;
    sub_robot_status.enabled = false;
    sub_robot_state_actual.enabled = false;
    sub_debug_foot_pos.enabled = false;
    if (rc_log_network_packets) Serial.println("[RC] All telemetry subscriptions cleared.");
}

static void parseAndStoreMobileAppInput(const JsonDocument& doc) {
  const char* id = doc["id"];
  const char* type = doc["type"];
  if (!id || !type) return;

  if (strcmp(type, "JOYSTICK") == 0 && strcmp(id, "walk_stick") == 0) {
    app_raw_joystick_x = doc["x"] | 0.0f;
    app_raw_joystick_y = doc["y"] | 0.0f;
  } else if (strcmp(type, "STEERING_WHEEL") == 0 && strcmp(id, "steering_wheel") == 0) {
    app_raw_steering_angle_deg = doc["angle"] | 0.0f;
  } else if (strcmp(type, "SWITCH") == 0 && strcmp(id, "en_phone") == 0) {
    bool new_app_enabled_state = doc["state"] | false;
    if (app_is_enabled_by_switch != new_app_enabled_state) { // State changed
        app_is_enabled_by_switch = new_app_enabled_state;
        walkCycleRunning = app_is_enabled_by_switch; // Directly control walkCycleRunning

        if (rc_log_network_packets) Serial.printf("Mobile App Control Switch: %s. Walk Active: %s\n", 
                                                app_is_enabled_by_switch ? "ON" : "OFF",
                                                walkCycleRunning ? "ON" : "OFF");
        if (!app_is_enabled_by_switch) { 
            active_target_vx_factor = 0.0f; active_target_vy_factor = 0.0f; active_target_yaw_factor = 0.0f;
            active_offset_x_direction = 0.0f; active_offset_y_direction = 0.0f; active_offset_z_direction = 0.0f;
            active_pitch_direction = 0.0f; active_roll_direction = 0.0f; active_body_yaw_direction = 0.0f;
            active_centering_xy = false; active_centering_orientation = false;
            // bodyVelocity and bodyAngularVelocityYaw will naturally ramp down due to integrateRobotState
        }
    }
  } else if (strcmp(type, "BUTTON") == 0) {
    bool pressed = false; 
    JsonVariantConst state_var = doc["state"];
    if(state_var.is<const char*>()) pressed = (state_var.as<const char*>()[0] == 'P'); 
    else if(state_var.is<bool>()) pressed = state_var.as<bool>(); 

    if (strcmp(id, "center_xy") == 0) app_hold_center_xy = pressed;
    else if (strcmp(id, "center_rot") == 0) app_hold_center_rot = pressed;
    else if (strcmp(id, "yaw+") == 0) app_hold_yaw_plus = pressed;
    else if (strcmp(id, "yaw-") == 0) app_hold_yaw_minus = pressed;
    else if (strcmp(id, "up") == 0) app_hold_body_z_up = pressed;
    else if (strcmp(id, "down") == 0) app_hold_body_z_down = pressed;
  } else if (strcmp(type, "DPAD") == 0) {
    bool pressed = false;
    JsonVariantConst state_var = doc["state"];
    if(state_var.is<const char*>()) pressed = (state_var.as<const char*>()[0] == 'P');
    else if(state_var.is<bool>()) pressed = state_var.as<bool>();
    
    const char* button = doc["button"];
    if (!button) return;
    if (strcmp(id, "dpad_xy") == 0) {
      if (strcmp(button, "UP") == 0) app_hold_dpad_xy_up = pressed;
      else if (strcmp(button, "DOWN") == 0) app_hold_dpad_xy_down = pressed;
      else if (strcmp(button, "LEFT") == 0) app_hold_dpad_xy_left = pressed;
      else if (strcmp(button, "RIGHT") == 0) app_hold_dpad_xy_right = pressed;
    } else if (strcmp(id, "dpad_rot") == 0) {
      if (strcmp(button, "UP") == 0) app_hold_dpad_rot_pitch_down = pressed; 
      else if (strcmp(button, "DOWN") == 0) app_hold_dpad_rot_pitch_up = pressed; 
      else if (strcmp(button, "LEFT") == 0) app_hold_dpad_rot_roll_left = pressed;
      else if (strcmp(button, "RIGHT") == 0) app_hold_dpad_rot_roll_right = pressed;
    }
  } else if (strcmp(type, "SLIDER") == 0) {
    float value = doc["value"] | 0.0f;
    if (strcmp(id, "step_t") == 0) walkParams.stepTime = clampf(value, 0.1f, 5.0f);
    else if (strcmp(id, "step_h") == 0) walkParams.stepHeight = clampf(value, 0.0f, 10.0f);
  }
}

static void translateMobileInputsToActiveIntents() {
  active_target_vy_factor = clampf(app_raw_joystick_y, -1.0f, 1.0f);
  active_target_vx_factor = clampf(app_raw_joystick_x, -1.0f, 1.0f);

  if (fabs(app_raw_steering_angle_deg) > 0.1f) {
    active_target_yaw_factor = clampf(app_raw_steering_angle_deg / 90.0f, -1.0f, 1.0f);
  } else if (app_hold_yaw_plus) {
    active_target_yaw_factor = 1.0f;
  } else if (app_hold_yaw_minus) {
    active_target_yaw_factor = -1.0f;
  } else {
    active_target_yaw_factor = 0.0f;
  }

  active_offset_y_direction = (app_hold_dpad_xy_up ? 1.0f : 0.0f) + (app_hold_dpad_xy_down ? -1.0f : 0.0f);
  active_offset_x_direction = (app_hold_dpad_xy_left ? -1.0f : 0.0f) + (app_hold_dpad_xy_right ? 1.0f : 0.0f);
  active_offset_z_direction = (app_hold_body_z_up ? 1.0f : 0.0f) + (app_hold_body_z_down ? -1.0f : 0.0f);

  active_pitch_direction = (app_hold_dpad_rot_pitch_up ? 1.0f : 0.0f) + (app_hold_dpad_rot_pitch_down ? -1.0f : 0.0f);
  active_roll_direction = (app_hold_dpad_rot_roll_left ? -1.0f : 0.0f) + (app_hold_dpad_rot_roll_right ? 1.0f : 0.0f);
  
  active_centering_xy = app_hold_center_xy;
  active_centering_orientation = app_hold_center_rot;
}

static void integrateRobotState(float dt) {
    float raw_target_vx = active_target_vx_factor * cfg_max_linear_speed_cms;
    float raw_target_vy = active_target_vy_factor * cfg_max_linear_speed_cms;
    float target_yaw_speed = -active_target_yaw_factor * cfg_max_yaw_rate_rads;

    float target_xy_speed_sq = raw_target_vx * raw_target_vx + raw_target_vy * raw_target_vy;
    float target_vx_final = raw_target_vx;
    float target_vy_final = raw_target_vy;

    if (target_xy_speed_sq > (cfg_max_linear_speed_cms * cfg_max_linear_speed_cms) && target_xy_speed_sq > 0.001f) {
        float target_xy_speed_current_magnitude = sqrtf(target_xy_speed_sq);
        float scale = cfg_max_linear_speed_cms / target_xy_speed_current_magnitude;
        target_vx_final = raw_target_vx * scale;
        target_vy_final = raw_target_vy * scale;
    }

    float diff_vx = target_vx_final - bodyVelocity.x;
    float current_accel_x_limit_s2;
    if (fabsf(diff_vx) < 0.01f) { 
         current_accel_x_limit_s2 = cfg_linear_deceleration_cm_s2; 
    } else if ( (diff_vx > 0 && bodyVelocity.x >= -0.01f && target_vx_final > bodyVelocity.x) || 
                (diff_vx < 0 && bodyVelocity.x <=  0.01f && target_vx_final < bodyVelocity.x) ) { 
        current_accel_x_limit_s2 = cfg_linear_acceleration_cm_s2;
    } else { 
        current_accel_x_limit_s2 = cfg_linear_deceleration_cm_s2;
    }
    float accel_x_step = current_accel_x_limit_s2 * dt;
    bodyVelocity.x += clampf(diff_vx, -accel_x_step, accel_x_step);

    float diff_vy = target_vy_final - bodyVelocity.y;
    float current_accel_y_limit_s2;
    if (fabsf(diff_vy) < 0.01f) {
         current_accel_y_limit_s2 = cfg_linear_deceleration_cm_s2;
    } else if ( (diff_vy > 0 && bodyVelocity.y >= -0.01f && target_vy_final > bodyVelocity.y) ||
                (diff_vy < 0 && bodyVelocity.y <=  0.01f && target_vy_final < bodyVelocity.y) ) {
        current_accel_y_limit_s2 = cfg_linear_acceleration_cm_s2;
    } else {
        current_accel_y_limit_s2 = cfg_linear_deceleration_cm_s2;
    }
    float accel_y_step = current_accel_y_limit_s2 * dt;
    bodyVelocity.y += clampf(diff_vy, -accel_y_step, accel_y_step);

    bodyVelocity.x = clampf(bodyVelocity.x, -cfg_max_linear_speed_cms, cfg_max_linear_speed_cms);
    bodyVelocity.y = clampf(bodyVelocity.y, -cfg_max_linear_speed_cms, cfg_max_linear_speed_cms);
    
    float current_speed_sq = bodyVelocity.x * bodyVelocity.x + bodyVelocity.y * bodyVelocity.y;
    if (current_speed_sq > (cfg_max_linear_speed_cms * cfg_max_linear_speed_cms) && current_speed_sq > 0.001f) {
        float current_speed_mag = sqrtf(current_speed_sq);
        float scale_final = cfg_max_linear_speed_cms / current_speed_mag;
        bodyVelocity.x *= scale_final;
        bodyVelocity.y *= scale_final;
    }

    float diff_yaw_speed = target_yaw_speed - bodyAngularVelocityYaw;
    float current_accel_yaw_limit_s2;
    if (fabsf(diff_yaw_speed) < 0.001f) { 
         current_accel_yaw_limit_s2 = cfg_yaw_deceleration_rad_s2;
    } else if ( (diff_yaw_speed > 0 && bodyAngularVelocityYaw >= -0.001f && target_yaw_speed > bodyAngularVelocityYaw) ||
                (diff_yaw_speed < 0 && bodyAngularVelocityYaw <=  0.001f && target_yaw_speed < bodyAngularVelocityYaw) ) {
        current_accel_yaw_limit_s2 = cfg_yaw_acceleration_rad_s2;
    } else {
        current_accel_yaw_limit_s2 = cfg_yaw_deceleration_rad_s2;
    }
    float accel_yaw_step = current_accel_yaw_limit_s2 * dt;
    bodyAngularVelocityYaw += clampf(diff_yaw_speed, -accel_yaw_step, accel_yaw_step);
    bodyAngularVelocityYaw = clampf(bodyAngularVelocityYaw, -cfg_max_yaw_rate_rads, cfg_max_yaw_rate_rads);

  if (active_centering_xy) {
    float linear_centering_step = cfg_pose_adjust_linear_cms * dt;
    if (fabs(bodyPositionOffset.x) < 0.1f) bodyPositionOffset.x = 0.0f;
    else if (bodyPositionOffset.x > 0) bodyPositionOffset.x -= linear_centering_step;
    else bodyPositionOffset.x += linear_centering_step;

    if (fabs(bodyPositionOffset.y) < 0.1f) bodyPositionOffset.y = 0.0f;
    else if (bodyPositionOffset.y > 0) bodyPositionOffset.y -= linear_centering_step;
    else bodyPositionOffset.y += linear_centering_step;
  } else {
    bodyPositionOffset.x += active_offset_x_direction * cfg_pose_adjust_linear_cms * dt;
    bodyPositionOffset.y += active_offset_y_direction * cfg_pose_adjust_linear_cms * dt;
  }
  bodyPositionOffset.z += active_offset_z_direction * cfg_pose_adjust_linear_cms * dt;

  if (active_centering_orientation) {
    float slerp_factor = 3.0f * dt; 
    bodyOrientation = slerp(bodyOrientation, Quaternion::identity(), slerp_factor);
    if (dot(bodyOrientation, Quaternion::identity()) > 0.99999f) {
        bodyOrientation = Quaternion::identity();
    }
  } else {
    Quaternion rot_delta = Quaternion::identity();
    if (fabs(active_pitch_direction) > 0.01) {
      rot_delta = rot_delta * Quaternion::from_axis_angle({1.0f, 0.0f, 0.0f}, active_pitch_direction * cfg_pose_adjust_angular_rads * dt);
    }
    if (fabs(active_roll_direction) > 0.01) {
      rot_delta = rot_delta * Quaternion::from_axis_angle({0.0f, 1.0f, 0.0f}, active_roll_direction * cfg_pose_adjust_angular_rads * dt);
    }
    if (fabs(active_body_yaw_direction) > 0.01) {
      rot_delta = rot_delta * Quaternion::from_axis_angle({0.0f, 0.0f, 1.0f}, active_body_yaw_direction * cfg_pose_adjust_angular_rads * dt);
    }
    bodyOrientation = bodyOrientation * rot_delta;
    bodyOrientation.normalize();
  }
}

static void sendConfiguredTelemetry() {
  if (!global_telemetry_target_known) return;

  uint32_t current_ms = millis();
  
  if (sub_battery.enabled && (current_ms - sub_battery.last_sent_ms >= sub_battery.interval_ms)) {
    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry_data"; 
    doc["source"] = "esp32_hexapod";
    JsonObject payload = doc.createNestedObject("payload");
    JsonObject battery_data = payload.createNestedObject("battery");
    battery_data["voltage_v"] = readBatteryVoltage();

    if (network_comms_send_json_to_ip_tcp(global_telemetry_destination_ip, doc)) {
        if (rc_log_network_packets) Serial.println("[RC] TX Battery (TCP)");
        sub_battery.last_sent_ms = current_ms;
    } else if (rc_log_network_packets) Serial.println("[RC WARN] Failed to send Battery (TCP)");
  }

  if (sub_robot_status.enabled && (current_ms - sub_robot_status.last_sent_ms >= sub_robot_status.interval_ms)) {
    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry_data";
    doc["source"] = "esp32_hexapod";
    JsonObject payload = doc.createNestedObject("payload");
    JsonObject status_data = payload.createNestedObject("robot_status");
    status_data["wifi_rssi_dbm"] = WiFi.RSSI();
    status_data["active_controller_hint"] = app_is_enabled_by_switch ? "mobile_app" : "python_gui";
    
    if (network_comms_send_json_to_ip_tcp(global_telemetry_destination_ip, doc)) {
        if (rc_log_network_packets) Serial.println("[RC] TX RobotStatus (TCP)");
        sub_robot_status.last_sent_ms = current_ms;
    } else if (rc_log_network_packets) Serial.println("[RC WARN] Failed to send RobotStatus (TCP)");
  }

  bool send_udp_telemetry_this_cycle = false;
  if (sub_robot_state_actual.enabled && (current_ms - sub_robot_state_actual.last_sent_ms >= sub_robot_state_actual.interval_ms)) {
    send_udp_telemetry_this_cycle = true;
  }
  if (sub_debug_foot_pos.enabled && (current_ms - sub_debug_foot_pos.last_sent_ms >= sub_debug_foot_pos.interval_ms)) {
    send_udp_telemetry_this_cycle = true;
  }

  if (send_udp_telemetry_this_cycle) {
    DynamicJsonDocument telemetry_doc(768); 

    telemetry_doc["type"] = "robot_state_telemetry"; 
    telemetry_doc["source"] = "esp32_hexapod";

    JsonObject payload = telemetry_doc.createNestedObject("payload");
    bool payload_has_data = false;

    if (sub_robot_state_actual.enabled) {
        JsonObject loco_actual = payload.createNestedObject("locomotion_actual");
        loco_actual["velocity_x_cms"] = bodyVelocity.x;
        loco_actual["velocity_y_cms"] = bodyVelocity.y;
        loco_actual["angular_velocity_yaw_rads"] = bodyAngularVelocityYaw;

        JsonObject pose_actual = payload.createNestedObject("body_pose_actual");
        JsonObject pos_offset = pose_actual.createNestedObject("position_offset_cm");
        pos_offset["x"] = bodyPositionOffset.x; pos_offset["y"] = bodyPositionOffset.y; pos_offset["z"] = bodyPositionOffset.z;
        JsonObject orient_q = pose_actual.createNestedObject("orientation_quat");
        orient_q["w"] = bodyOrientation.w; orient_q["x"] = bodyOrientation.x; orient_q["y"] = bodyOrientation.y; orient_q["z"] = bodyOrientation.z;

        JsonObject gait_actual = payload.createNestedObject("gait_actual");
        gait_actual["step_height_cm"] = walkParams.stepHeight;
        gait_actual["step_time_s"] = walkParams.stepTime;
        gait_actual["walk_active"] = walkCycleRunning;
        payload_has_data = true;
    }

    if (sub_debug_foot_pos.enabled) {
        JsonArray foot_pos_walk_array = payload.createNestedArray("debug_foot_pos_walk_cm");
        for (int i = 0; i < LEG_COUNT; ++i) {
            JsonObject leg_pos_obj = foot_pos_walk_array.createNestedObject();
            leg_pos_obj["x"] = round(legCycleData[i].currentPosition.x * 100.0f) / 100.0f;
            leg_pos_obj["y"] = round(legCycleData[i].currentPosition.y * 100.0f) / 100.0f;
            leg_pos_obj["z"] = round(legCycleData[i].currentPosition.z * 100.0f) / 100.0f;
        }
        payload_has_data = true;
    }

    if (payload_has_data) {
        if(network_comms_send_json_to_ip_port_udp(global_telemetry_destination_ip, global_telemetry_destination_udp_port, telemetry_doc)) {
            if (sub_robot_state_actual.enabled && (current_ms - sub_robot_state_actual.last_sent_ms >= sub_robot_state_actual.interval_ms)) {
                 sub_robot_state_actual.last_sent_ms = current_ms;
            }
            if (sub_debug_foot_pos.enabled && (current_ms - sub_debug_foot_pos.last_sent_ms >= sub_debug_foot_pos.interval_ms)) {
                 sub_debug_foot_pos.last_sent_ms = current_ms;
            }
        } else if (rc_log_network_packets) Serial.println("[RC WARN] Failed to send State/FootPos (UDP)");
    }
  }
}

static void sendDiscoveryBeacon() {
    if (WiFi.status() != WL_CONNECTED) return;

    StaticJsonDocument<256> beacon_doc;
    beacon_doc["type"] = "hexapod_discovery_beacon";
    beacon_doc["id"] = "Hexapod_ESP32_Ossian"; // Unique ID for your hexapod
    beacon_doc["ip"] = network_comms_get_local_ip().toString();
    beacon_doc["tcp_port"] = TCP_LISTEN_PORT;
    beacon_doc["udp_port"] = UDP_LISTEN_PORT; // The main command UDP port

    if (network_comms_send_broadcast_udp_json(DISCOVERY_BROADCAST_PORT, beacon_doc)) {
        if (rc_log_network_packets) Serial.printf("[RC Beacon] Sent discovery beacon to port %u.\n", DISCOVERY_BROADCAST_PORT);
    } else {
        if (rc_log_network_packets) Serial.printf("[RC Beacon WARN] Failed to send discovery beacon to port %u.\n", DISCOVERY_BROADCAST_PORT);
    }
}

static void calculate_and_update_base_foot_positions_from_abstract_config() {
    Vec3 sym_base_xy[LEG_COUNT] = {
        { cfg_leg_geom_front_corner_x_cm, -cfg_leg_geom_front_corner_y_cm, 0.0f},
        { cfg_leg_geom_middle_side_x_cm,   0.0f                          , 0.0f},
        { cfg_leg_geom_front_corner_x_cm,  cfg_leg_geom_front_corner_y_cm, 0.0f},
        {-cfg_leg_geom_front_corner_x_cm, -cfg_leg_geom_front_corner_y_cm, 0.0f},
        {-cfg_leg_geom_middle_side_x_cm,   0.0f                          , 0.0f},
        {-cfg_leg_geom_front_corner_x_cm,  cfg_leg_geom_front_corner_y_cm, 0.0f}
    };

    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        float base_x = sym_base_xy[i].x;
        float base_y = sym_base_xy[i].y;
        
        float ext_cm = 0.0f;
        if (i == 0 || i == 2 || i == 3 || i == 5) { 
            ext_cm = cfg_leg_geom_corner_ext_cm;
        } else { 
            ext_cm = cfg_leg_geom_middle_ext_cm;
        }
        
        float angle_rad = legMountingAngle[i]; 

        baseFootPositionWalk[i].x = base_x + ext_cm * cosf(angle_rad);
        baseFootPositionWalk[i].y = base_y + ext_cm * sinf(angle_rad);
        baseFootPositionWalk[i].z = 0.0f; 
    }
    if (rc_log_network_packets) { 
         Serial.println("[RC] Base foot positions recalculated from abstract geometry.");
    }
}