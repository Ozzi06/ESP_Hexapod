// remote_control.cpp
#include "remote_control.h"
#include "network_comms.h"  // NEW: For TCP/UDP communications
#include "math_utils.h"
#include "servo_angles.h"     // For setupPwm in reinitialize_pwm
#include "battery.h"          // For readBatteryVoltage()
#include "robot_spec.h"       // For current leg positions, LEG_COUNT etc.
#include "walkcycle.h"        // For walkParams, legCycleData, updateWalkCycle

// --- Configuration & Global State for Remote Control ---
static bool rc_log_network_packets = false; // Renamed to avoid conflict if network_comms has its own log flag
static unsigned long rc_last_packet_received_ms = 0;

// --- ESP32 Owned Configuration Parameters (set by "config_update") ---
static float cfg_max_linear_speed_cms = 8.0f;
static float cfg_max_yaw_rate_rads = 0.3f;
static float cfg_pose_adjust_linear_cms = 2.0f;
static float cfg_pose_adjust_angular_rads = 0.26f; // ~15 deg/s

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
static IPAddress global_telemetry_destination_ip;         // Set by client_settings
static uint16_t global_telemetry_destination_udp_port = 0; // Set by client_settings
static bool global_telemetry_target_known = false;      // True if IP and Port are set

struct TelemetrySubscription {
  bool enabled = false;
  uint32_t interval_ms = 1000;
  uint32_t last_sent_ms = 0;
};
// These are global subscriptions for the single telemetry target (GUI)
static TelemetrySubscription sub_battery;          // TCP
static TelemetrySubscription sub_robot_status;     // TCP
static TelemetrySubscription sub_robot_state_actual; // UDP
static TelemetrySubscription sub_debug_foot_pos;   // UDP


// --- Forward Declarations for internal functions ---
static void process_json_packet_internal(const JsonDocument& doc, IPAddress source_ip, bool is_tcp, WiFiClient tcp_client);
static void processLocomotionIntent(JsonObjectConst payload);
static void processPoseAdjustIntent(JsonObjectConst payload);
static void processCenteringIntent(JsonObjectConst payload);
static void processGaitCommand(JsonObjectConst payload);
static void processConfigUpdate(JsonObjectConst payload);
static void processClientSettings(JsonObjectConst payload, IPAddress source_ip);
static void processRequestFullState(IPAddress reply_to_ip); // reply_to_ip is the GUI's IP
static void processDisconnectNotice(JsonObjectConst payload);
static void processSystemCommand(JsonObjectConst payload);

static void parseAndStoreMobileAppInput(const JsonDocument& doc);
static void translateMobileInputsToActiveIntents();
static void integrateRobotState(float dt);
static void sendConfiguredTelemetry();
static void clear_all_subscriptions(); // Helper to disable all telemetry

// --- Callbacks for network_comms ---

/**
 * @brief Callback invoked by network_comms when a JSON packet is parsed.
 */
static void json_packet_received_callback(const JsonDocument& doc, IPAddress source_ip, uint16_t source_port, bool is_tcp, WiFiClient tcp_client) {
    rc_last_packet_received_ms = millis(); // Update activity timestamp

    if (rc_log_network_packets) {
        Serial.printf("[%s RX %s:%u] ", is_tcp ? "TCP" : "UDP", source_ip.toString().c_str(), source_port);
        // serializeJsonPretty(doc, Serial); // Already logged by network_comms if its internal log is on
        // Serial.println();
    }
    process_json_packet_internal(doc, source_ip, is_tcp, tcp_client);
}

/**
 * @brief Callback invoked by network_comms on abrupt TCP client disconnection.
 */
static void tcp_client_abrupt_disconnect_callback(IPAddress disconnected_client_ip) {
    if (rc_log_network_packets) {
        Serial.printf("[RC] TCP client %s abruptly disconnected.\n", disconnected_client_ip.toString().c_str());
    }
    if (global_telemetry_target_known && global_telemetry_destination_ip == disconnected_client_ip) {
        Serial.println("[RC] Disconnected client was the telemetry target. Clearing subscriptions.");
        clear_all_subscriptions();
        global_telemetry_target_known = false;
        global_telemetry_destination_ip = IPAddress(); // Clear IP
    }
}

// --- Public Functions ---

void setupRemoteControl() {
  Serial.println("--- Remote Control Mode Setup ---");
  setupWalkcycle();

  rc_log_network_packets = false; // Default logging state for remote_control specific logs
  rc_last_packet_received_ms = millis();

  // Default config speeds
  cfg_max_linear_speed_cms = 8.0f;
  cfg_max_yaw_rate_rads = 0.3f;
  cfg_pose_adjust_linear_cms = 2.0f;
  cfg_pose_adjust_angular_rads = 15.0f * M_PI / 180.0f;

  // Default states for subscriptions (before a client configures them)
  // These will be enabled/disabled by client_settings message
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

  // 1. Handle network communications (receives packets, calls callbacks)
  network_comms_handle();

  // 2. Process Serial Commands
  if (Serial.available() > 0) {
    char command = toupper(Serial.read());
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) { Serial.read(); } // Clear buffer
    Serial.print("RemoteMode CMD> "); Serial.println(command);
    switch (command) {
      case 'L': rc_log_network_packets = !rc_log_network_packets; Serial.printf("RC Network Packet Logging: %s\n", rc_log_network_packets ? "ON" : "OFF"); break;
      case 'X': walkCycleRunning = false; bodyVelocity = {0,0,0}; bodyAngularVelocityYaw = 0; return false; // Exit mode
      case 'H': case '?': printRemoteControlSerialHelp(); break;
      default: Serial.println("Unknown RemoteMode CMD."); break;
    }
  }

  // 3. Check for Global Connection Timeout (based on any packet received via network_comms)
  if (millis() - rc_last_packet_received_ms > GLOBAL_PACKET_TIMEOUT_MS) {
    Serial.println("[RC WARN] GLOBAL PACKET TIMEOUT! Halting motion.");
    active_target_vx_factor = 0.0f; active_target_vy_factor = 0.0f; active_target_yaw_factor = 0.0f;
    active_offset_x_direction = 0.0f; active_offset_y_direction = 0.0f; active_offset_z_direction = 0.0f;
    active_pitch_direction = 0.0f; active_roll_direction = 0.0f; active_body_yaw_direction = 0.0f;
    active_centering_xy = false; active_centering_orientation = false;
    walkCycleRunning = false;
    rc_last_packet_received_ms = currentTimeMs; // Prevent spamming
  }

  // 4. If mobile app is enabled, translate its raw inputs to active_intents
  if (app_is_enabled_by_switch) {
    translateMobileInputsToActiveIntents();
  }

  // 5. Integrate all active intents and configurations into robot state
  integrateRobotState(dt);

  // 6. Update Walk Cycle if active
  if (walkCycleRunning) {
    updateWalkCycle(dt);
  }

  // 7. Send Configured Telemetry
  if (global_telemetry_target_known) {
    sendConfiguredTelemetry();
  }

  return true; // Continue in this mode
}

void printRemoteControlSerialHelp() {
  Serial.println("\n--- Remote Control Mode Serial ---");
  Serial.println("  L - Toggle RC Network Packet Logging");
  Serial.println("  X - Exit to Main Menu");
  Serial.println("  H / ? - Display this help");
  Serial.println("--------------------------------");
}


// --- Internal JSON Packet Processing Logic ---
static void process_json_packet_internal(const JsonDocument& doc, IPAddress source_ip, bool is_tcp, WiFiClient tcp_client) {
  JsonVariantConst type_variant = doc["type"];
  if (!type_variant.is<const char*>()) {
    if (rc_log_network_packets) Serial.println("[RC] JSON no 'type' string.");
    return;
  }
  const char* type = type_variant.as<const char*>();

  // Check for mobile app packets (always check these first)
  // These are identified by having an "id" field and specific types.
  if (doc.containsKey("id") &&
      (strcmp(type, "JOYSTICK") == 0 || strcmp(type, "BUTTON") == 0 ||
       strcmp(type, "DPAD") == 0 || strcmp(type, "SWITCH") == 0 ||
       strcmp(type, "STEERING_WHEEL") == 0 || strcmp(type, "SLIDER") == 0)) {
    parseAndStoreMobileAppInput(doc);
    return;
  }

  // Standard GUI/Control commands usually have a "payload"
  JsonVariantConst payload_variant = doc["payload"];
  // Some commands might not have a payload (e.g. request_full_state might just have reply_to_ip)
  
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
      JsonVariantConst reply_ip_var = doc["reply_to_ip"]; // No "payload" wrapper for this message
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
    //do nothing
  }
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
  if (payload.containsKey("walk_active")) {
    walkCycleRunning = payload["walk_active"].as<bool>();
     if (rc_log_network_packets) Serial.printf("[RC] Walk Active set to: %s\n", walkCycleRunning ? "true" : "false");
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
  if (payload.containsKey("base_foot_positions_walk_cm")) {
    JsonArrayConst positions = payload["base_foot_positions_walk_cm"].as<JsonArrayConst>();
    if (positions && positions.size() == LEG_COUNT) {
      if (rc_log_network_packets) Serial.println("  Base foot positions updating...");
      for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        JsonObjectConst leg_pos = positions[i].as<JsonObjectConst>();
        if (leg_pos) {
          baseFootPositionWalk[i].x = leg_pos["x"] | baseFootPositionWalk[i].x;
          baseFootPositionWalk[i].y = leg_pos["y"] | baseFootPositionWalk[i].y;
          baseFootPositionWalk[i].z = leg_pos["z"] | baseFootPositionWalk[i].z;
        }
      }
    } else if (rc_log_network_packets) Serial.println("  [WARN] base_foot_positions_walk_cm missing or wrong size.");
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
            // Potentially clear global_telemetry_target_known = false; if config is partial/invalid
        }
        
        JsonObjectConst udp_subs = udp_config["subscriptions"].as<JsonObjectConst>();
        if (udp_subs) {
            if (rc_log_network_packets) Serial.println("  Updating UDP subscriptions:");
            // Robot State Actual (UDP)
            if (udp_subs["robot_state_actual"].is<JsonObjectConst>()) {
                sub_robot_state_actual.enabled = udp_subs["robot_state_actual"]["enabled"] | false;
                sub_robot_state_actual.interval_ms = udp_subs["robot_state_actual"]["interval_ms"] | 200;
                sub_robot_state_actual.last_sent_ms = 0; // Force send on next cycle if enabled
                if (rc_log_network_packets) Serial.printf("    RobotStateActual (UDP): %s, %ums\n", sub_robot_state_actual.enabled?"ON":"OFF", sub_robot_state_actual.interval_ms);
            }
            // Debug Foot Pos (UDP)
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
        // Battery (TCP)
        if (tcp_subs_config["battery"].is<JsonObjectConst>()) {
            sub_battery.enabled = tcp_subs_config["battery"]["enabled"] | false;
            sub_battery.interval_ms = tcp_subs_config["battery"]["interval_ms"] | 1000;
            sub_battery.last_sent_ms = 0;
            if (rc_log_network_packets) Serial.printf("    Battery (TCP): %s, %ums\n", sub_battery.enabled?"ON":"OFF", sub_battery.interval_ms);
        }
        // Robot Status (TCP)
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

    DynamicJsonDocument full_state_doc(1024); // Make it large enough
    full_state_doc["type"] = "full_state_response";
    full_state_doc["source"] = "esp32_hexapod";
    
    JsonObject payload = full_state_doc.createNestedObject("payload");

    // Configurable parameters
    JsonObject speeds = payload.createNestedObject("max_speeds");
    speeds["linear_cms"] = cfg_max_linear_speed_cms;
    speeds["yaw_rads"] = cfg_max_yaw_rate_rads;

    JsonObject pose_speeds = payload.createNestedObject("pose_adjust_speeds");
    pose_speeds["linear_cms"] = cfg_pose_adjust_linear_cms;
    pose_speeds["angular_rads"] = cfg_pose_adjust_angular_rads;

    JsonObject gait = payload.createNestedObject("gait_params");
    gait["step_height_cm"] = walkParams.stepHeight;
    gait["step_time_s"] = walkParams.stepTime;
    
    payload["walk_active"] = walkCycleRunning; // Current gait command state

    JsonArray base_pos_array = payload.createNestedArray("base_foot_positions_walk_cm");
    for(int i=0; i<LEG_COUNT; ++i) {
        JsonObject leg_obj = base_pos_array.createNestedObject();
        leg_obj["x"] = baseFootPositionWalk[i].x;
        leg_obj["y"] = baseFootPositionWalk[i].y;
        leg_obj["z"] = baseFootPositionWalk[i].z;
    }

    // Current robot state
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

    // Send it via TCP using network_comms
    if (!network_comms_send_json_to_ip_tcp(reply_to_ip, full_state_doc)) {
        if (rc_log_network_packets) Serial.printf("  [WARN] Failed to send full_state_response to %s\n", reply_to_ip.toString().c_str());
    } else if (rc_log_network_packets) Serial.printf("  Sent full_state_response to %s\n", reply_to_ip.toString().c_str());
}

static void processDisconnectNotice(JsonObjectConst payload) {
    JsonVariantConst ip_var = payload["source_ip"];
    if (ip_var.is<const char*>()) {
        IPAddress source_ip;
        if (source_ip.fromString(ip_var.as<const char*>())) {
            if (rc_log_network_packets) Serial.printf("[RC] Processing 'disconnect_notice' from %s\n", source_ip.toString().c_str());
            if (global_telemetry_target_known && global_telemetry_destination_ip == source_ip) {
                Serial.println("  Client was telemetry target. Clearing subscriptions.");
                clear_all_subscriptions();
                global_telemetry_target_known = false;
                global_telemetry_destination_ip = IPAddress();
            }
            network_comms_close_client_by_ip(source_ip); // Tell network_comms to close this TCP client
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

static void clear_all_subscriptions() {
    sub_battery.enabled = false;
    sub_robot_status.enabled = false;
    sub_robot_state_actual.enabled = false;
    sub_debug_foot_pos.enabled = false;
    if (rc_log_network_packets) Serial.println("[RC] All telemetry subscriptions cleared.");
}

// --- Mobile App Input Processing ( Largely unchanged from your version) ---
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
    app_is_enabled_by_switch = doc["state"] | false;
    if (rc_log_network_packets) Serial.printf("Mobile App Control Switch: %s\n", app_is_enabled_by_switch ? "ON" : "OFF");
    if (!app_is_enabled_by_switch) { 
        active_target_vx_factor = 0.0f; active_target_vy_factor = 0.0f; active_target_yaw_factor = 0.0f;
        active_offset_x_direction = 0.0f; active_offset_y_direction = 0.0f; active_offset_z_direction = 0.0f;
        active_pitch_direction = 0.0f; active_roll_direction = 0.0f; active_body_yaw_direction = 0.0f;
        active_centering_xy = false; active_centering_orientation = false;
    }
  } else if (strcmp(type, "BUTTON") == 0) {
    bool pressed = false; // Determine pressed state carefully
    JsonVariantConst state_var = doc["state"];
    if(state_var.is<const char*>()) pressed = (state_var.as<const char*>()[0] == 'P'); // "PRESS"
    else if(state_var.is<bool>()) pressed = state_var.as<bool>(); // if it's true/false

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
      if (strcmp(button, "UP") == 0) app_hold_dpad_rot_pitch_down = pressed; // Note: App Dpad UP for Pitch is often Pitch Nose DOWN on robot
      else if (strcmp(button, "DOWN") == 0) app_hold_dpad_rot_pitch_up = pressed; // App Dpad DOWN for Pitch is often Pitch Nose UP
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

// --- ESP32 State Integration (Unchanged from your version) ---
static void integrateRobotState(float dt) {
  float target_vx = active_target_vx_factor * cfg_max_linear_speed_cms;
  float target_vy = active_target_vy_factor * cfg_max_linear_speed_cms;
  float target_yaw = -active_target_yaw_factor * cfg_max_yaw_rate_rads;

  float accel_lin = 15.0f * cfg_max_linear_speed_cms * dt; 
  float accel_ang = 15.0f * cfg_max_yaw_rate_rads * dt;

  bodyVelocity.x = clampf(bodyVelocity.x + clampf(target_vx - bodyVelocity.x, -accel_lin, accel_lin), -cfg_max_linear_speed_cms, cfg_max_linear_speed_cms);
  bodyVelocity.y = clampf(bodyVelocity.y + clampf(target_vy - bodyVelocity.y, -accel_lin, accel_lin), -cfg_max_linear_speed_cms, cfg_max_linear_speed_cms);
  bodyAngularVelocityYaw = clampf(bodyAngularVelocityYaw + clampf(target_yaw - bodyAngularVelocityYaw, -accel_ang, accel_ang), -cfg_max_yaw_rate_rads, cfg_max_yaw_rate_rads);

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
  // TODO: Add clamping for bodyPositionOffset X, Y, Z

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

// --- Telemetry Sending (Modified to use network_comms) ---
static void sendConfiguredTelemetry() {
  if (!global_telemetry_target_known) return;

  uint32_t current_ms = millis();
  
  // --- Battery Telemetry (TCP) ---
  if (sub_battery.enabled && (current_ms - sub_battery.last_sent_ms >= sub_battery.interval_ms)) {
    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry_data"; // Using "telemetry_data" as per GUI expectation
    doc["source"] = "esp32_hexapod";
    JsonObject payload = doc.createNestedObject("payload");
    JsonObject battery_data = payload.createNestedObject("battery");
    battery_data["voltage_v"] = readBatteryVoltage();
    // battery_data["percentage"] = ... ; // If you have percentage calculation

    if (network_comms_send_json_to_ip_tcp(global_telemetry_destination_ip, doc)) {
        if (rc_log_network_packets) Serial.println("[RC] TX Battery (TCP)");
        sub_battery.last_sent_ms = current_ms;
    } else if (rc_log_network_packets) Serial.println("[RC WARN] Failed to send Battery (TCP)");
  }

  // --- Robot Status Telemetry (TCP) ---
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

  // --- Combined Robot State & Debug Foot Pos Telemetry (UDP) ---
  // Check if either UDP telemetry type is due
  bool send_udp_telemetry_this_cycle = false;
  if (sub_robot_state_actual.enabled && (current_ms - sub_robot_state_actual.last_sent_ms >= sub_robot_state_actual.interval_ms)) {
    send_udp_telemetry_this_cycle = true;
  }
  if (sub_debug_foot_pos.enabled && (current_ms - sub_debug_foot_pos.last_sent_ms >= sub_debug_foot_pos.interval_ms)) {
    send_udp_telemetry_this_cycle = true;
  }

  if (send_udp_telemetry_this_cycle) {
    DynamicJsonDocument telemetry_doc(768); // Reduced size a bit, adjust if necessary

    telemetry_doc["type"] = "robot_state_telemetry"; // GUI expects this type for this combined payload
    telemetry_doc["source"] = "esp32_hexapod";
    // telemetry_doc["timestamp_ms"] = current_ms; // Already in network_comms JSON wrapper if added there

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
            // Conditionally update last_sent_ms for those that were actually enabled and due
            if (sub_robot_state_actual.enabled && (current_ms - sub_robot_state_actual.last_sent_ms >= sub_robot_state_actual.interval_ms)) {
                 sub_robot_state_actual.last_sent_ms = current_ms;
            }
            if (sub_debug_foot_pos.enabled && (current_ms - sub_debug_foot_pos.last_sent_ms >= sub_debug_foot_pos.interval_ms)) {
                 sub_debug_foot_pos.last_sent_ms = current_ms;
            }
            // Optional: log successful UDP send (can be very verbose)
            // if (rc_log_network_packets && (millis() % 200 < 20)) Serial.println("[RC] TX State/FootPos (UDP)");

        } else if (rc_log_network_packets) Serial.println("[RC WARN] Failed to send State/FootPos (UDP)");
    }
  }
}