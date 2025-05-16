// remote_control.cpp
#include "remote_control.h"
#include "math_utils.h"
#include "servo_angles.h" // For setupPwm in reinitialize_pwm
#include "battery.h"      // Your header for readBatteryVoltage()
#include "robot_spec.h" // for current leg positions

// --- Configuration & Global State for Remote Control ---
static bool logUdpPackets = false;
static unsigned long lastPacketFromActiveControllerMs = 0;

// --- ESP32 Owned Configuration Parameters (set by Python GUI "config_update") ---
static float cfg_max_linear_speed_cms = 8.0f;    // Default, matches old GUI
static float cfg_max_yaw_rate_rads = 0.3f;       // Default
static float cfg_pose_adjust_linear_cms = 2.0f;  // Default
static float cfg_pose_adjust_angular_rads = 0.26f; // ~15 deg/s, Default

// --- ESP32 Owned Intent/Active State Variables (set by GUI/App intents) ---
// For locomotion
static float active_target_vx_factor = 0.0f; // -1.0 to 1.0
static float active_target_vy_factor = 0.0f; // -1.0 to 1.0
static float active_target_yaw_factor = 0.0f; // -1.0 to 1.0
// For pose adjustment (directions: -1, 0, 1)
static float active_offset_x_direction = 0.0f;
static float active_offset_y_direction = 0.0f;
static float active_offset_z_direction = 0.0f;
static float active_pitch_direction = 0.0f;
static float active_roll_direction = 0.0f;
static float active_body_yaw_direction = 0.0f; // For direct body orientation yaw adjust
// For centering
static bool active_centering_xy = false;
static bool active_centering_orientation = false;

// --- Mobile App Specific Input State ---
// These are intermediate states from raw mobile app packets,
// which then influence the 'active_*' variables above if mobile is in control.
static bool app_is_enabled_by_switch = false; // Controlled by "en_phone" switch
static float app_raw_joystick_x = 0.0f;
static float app_raw_joystick_y = 0.0f;
static float app_raw_steering_angle_deg = 0.0f;
// Mobile button hold states (true if currently held)
static bool app_hold_yaw_plus = false;
static bool app_hold_yaw_minus = false;
static bool app_hold_dpad_xy_up = false, app_hold_dpad_xy_down = false, app_hold_dpad_xy_left = false, app_hold_dpad_xy_right = false;
static bool app_hold_dpad_rot_pitch_up = false, app_hold_dpad_rot_pitch_down = false, app_hold_dpad_rot_roll_left = false, app_hold_dpad_rot_roll_right = false;
static bool app_hold_center_xy = false;
static bool app_hold_center_rot = false;
static bool app_hold_body_z_up = false;   // From "up" button
static bool app_hold_body_z_down = false; // From "down" (former "button")

// --- Telemetry State ---
static IPAddress telemetry_destination_ip;
static uint16_t telemetry_destination_port = 0;
static bool telemetry_target_known = false;

struct TelemetrySubscription {
  bool enabled = false;
  uint32_t interval_ms = 1000;
  uint32_t last_sent_ms = 0;
};
static TelemetrySubscription sub_battery;
static TelemetrySubscription sub_robot_status;
static TelemetrySubscription sub_robot_state_actual; // For sending back current pose, vel etc.
static TelemetrySubscription sub_debug_foot_pos;     // For foot position debug telemetry


// --- Internal Helper Function Declarations ---
void processJsonPacket(const JsonDocument& doc);
void processLocomotionIntent(JsonObjectConst payload);
void processPoseAdjustIntent(JsonObjectConst payload);
void processCenteringIntent(JsonObjectConst payload);
void processGaitCommand(JsonObjectConst payload);
void processConfigUpdate(JsonObjectConst payload);
void processClientSettings(JsonObjectConst payload);
void processSystemCommand(JsonObjectConst payload);

void parseAndStoreMobileAppInput(const JsonDocument& doc);
void translateMobileInputsToActiveIntents();

void integrateRobotState(float dt);
void sendConfiguredTelemetry();


// --- Public Functions ---
void setupRemoteControl() {
  Serial.println("--- Remote Control Mode Setup ---");
  setupWalkcycle(); // Initializes walkParams, legCycleData, walkCycleRunning = false;

  logUdpPackets = false;
  lastPacketFromActiveControllerMs = millis();

  // Initialize default config speeds (can be overridden by GUI)
  cfg_max_linear_speed_cms = 8.0f;
  cfg_max_yaw_rate_rads = 0.3f;
  cfg_pose_adjust_linear_cms = 2.0f;
  cfg_pose_adjust_angular_rads = 15.0f * M_PI / 180.0f; // Approx 0.26 rad/s

  // Initialize default telemetry subscriptions (can be overridden by GUI)
  sub_battery.enabled = true; sub_battery.interval_ms = 1000;
  sub_robot_status.enabled = true; sub_robot_status.interval_ms = 5000;
  sub_robot_state_actual.enabled = true; sub_robot_state_actual.interval_ms = 200; // Send pose/vel fairly often
  sub_debug_foot_pos.enabled = false; sub_debug_foot_pos.interval_ms = 200; //default off

  if (WiFi.status() == WL_CONNECTED) {
    if (udp.begin(UDP_LISTEN_PORT)) {
      Serial.printf("UDP Listener started on port %d for remote commands.\n", UDP_LISTEN_PORT);
    } else {
      Serial.println("[ERROR] Failed to start UDP Listener!");
    }
  } else {
    Serial.println("[WARN] WiFi not connected. UDP Listener not started.");
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
  if (dt <= 0.0f || dt > 0.1f) { dt = 0.02f; } // Clamp dt (50Hz nominal if dt is bad)

  // 1. Process Serial Commands
  if (Serial.available() > 0) {
    char command = toupper(Serial.read());
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) { Serial.read(); }
    Serial.print("RemoteMode CMD> "); Serial.println(command);
    switch (command) {
      case 'L': logUdpPackets = !logUdpPackets; Serial.printf("UDP Packet Logging: %s\n", logUdpPackets ? "ON" : "OFF"); break;
      case 'X': walkCycleRunning = false; bodyVelocity = {0,0,0}; bodyAngularVelocityYaw = 0; return false;
      case 'H': case '?': printRemoteControlSerialHelp(); break;
      default: Serial.println("Unknown RemoteMode CMD."); break;
    }
  }

  // 2. Process Incoming UDP Packets
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    lastPacketFromActiveControllerMs = currentTimeMs;
    StaticJsonDocument<1024> doc; // For control_state_update with base_foot_pos
                                 // For mobile app packets, a smaller doc would suffice if parsed separately.
    DeserializationError error = deserializeJson(doc, udp);
    udp.flush();

    if (error) {
      Serial.print(F("[ERROR] deserializeJson() failed: ")); Serial.println(error.f_str());
    } else {
      if (logUdpPackets) { Serial.println("RX UDP JSON:"); serializeJsonPretty(doc, Serial); Serial.println(); }
      processJsonPacket(doc); // Route to appropriate handler based on "type"
    }
  }

  // 3. Check for Global Connection Timeout
  if (currentTimeMs - lastPacketFromActiveControllerMs > GLOBAL_PACKET_TIMEOUT_MS) {
    Serial.println("[WARN] GLOBAL CONNECTION TIMEOUT! Halting motion.");
    active_target_vx_factor = 0.0f; active_target_vy_factor = 0.0f; active_target_yaw_factor = 0.0f;
    active_offset_x_direction = 0.0f; /* ...all active_offset/pitch/roll/yaw_direction to 0 ...*/
    active_centering_xy = false; active_centering_orientation = false;
    walkCycleRunning = false; // Also stop walk cycle
    // bodyVelocity and bodyAngularVelocityYaw will ramp down due to integrateRobotState
    lastPacketFromActiveControllerMs = currentTimeMs; // Prevent spamming this message
  }

  // 4. If mobile app is enabled, translate its raw inputs to our "active_intent" variables
  if (app_is_enabled_by_switch) {
    translateMobileInputsToActiveIntents();
  }
  // Note: If mobile app is NOT enabled, the "active_intent" variables retain values from Python GUI commands.

  // 5. Integrate all active intents and configurations into actual robot state (velocity, pose)
  integrateRobotState(dt);

  // 6. Update Walk Cycle if active
  if (walkCycleRunning) { // walkCycleRunning is a global bool set by gait_command or config_update
    updateWalkCycle(dt); // Uses global bodyVelocity, bodyOrientation, etc.
  }

  // 7. Send Configured Telemetry
  if (telemetry_target_known) {
    sendConfiguredTelemetry();
  }

  return true; // Continue in this mode
}

void printRemoteControlSerialHelp() {
  Serial.println("\n--- Remote Control Mode Serial ---");
  Serial.println("  L - Toggle UDP Packet Logging");
  Serial.println("  X - Exit to Main Menu");
  Serial.println("  H / ? - Display this help");
  Serial.println("--------------------------------");
}

void processJsonPacket(const JsonDocument& doc) {
  JsonVariantConst type_variant = doc["type"];
  if (!type_variant.is<const char*>()) { Serial.println("[WARN] JSON no 'type' string."); return; }
  const char* type = type_variant.as<const char*>();

  // Check for mobile app packets first by their unique structure/type fields
  if (doc.containsKey("id") && 
      (strcmp(type, "JOYSTICK") == 0 || strcmp(type, "BUTTON") == 0 ||
       strcmp(type, "DPAD") == 0 || strcmp(type, "SWITCH") == 0 ||
       strcmp(type, "STEERING_WHEEL") == 0 || strcmp(type, "SLIDER") == 0)) {
    parseAndStoreMobileAppInput(doc); // This just stores raw app inputs
    return; // Mobile app inputs are translated to active_intents later if app_is_enabled_by_switch
  }

  // Then check for Python GUI specific commands
  JsonVariantConst payload_variant = doc["payload"];
  if (!payload_variant.is<JsonObjectConst>()) {
    if (logUdpPackets) { Serial.print("[WARN] GUI command '"); Serial.print(type); Serial.println("' no valid object payload."); }
    return;
  }
  JsonObjectConst payload = payload_variant.as<JsonObjectConst>();

  if (strcmp(type, "locomotion_intent") == 0) processLocomotionIntent(payload);
  else if (strcmp(type, "pose_adjust_intent") == 0) processPoseAdjustIntent(payload);
  else if (strcmp(type, "centering_intent") == 0) processCenteringIntent(payload);
  else if (strcmp(type, "gait_command") == 0) processGaitCommand(payload);
  else if (strcmp(type, "config_update") == 0) processConfigUpdate(payload);
  else if (strcmp(type, "client_settings") == 0) processClientSettings(payload);
  else if (strcmp(type, "system_command") == 0) processSystemCommand(payload);
  else {
    if (logUdpPackets) { Serial.print("[WARN] Unknown GUI JSON msg type: "); Serial.println(type); }
  }
}

// --- Python GUI Command Handlers ---
void processLocomotionIntent(JsonObjectConst payload) {
  if (app_is_enabled_by_switch) return; // Ignore Python GUI loco if mobile active
  active_target_vx_factor = payload["target_vx_factor"] | 0.0f;
  active_target_vy_factor = payload["target_vy_factor"] | 0.0f;
  active_target_yaw_factor = payload["target_yaw_factor"] | 0.0f;
}

void processPoseAdjustIntent(JsonObjectConst payload) {
  if (app_is_enabled_by_switch) return; // Ignore Python GUI pose if mobile active
  active_offset_x_direction = payload["offset_x_active"] | 0.0f;
  active_offset_y_direction = payload["offset_y_active"] | 0.0f;
  active_offset_z_direction = payload["offset_z_active"] | 0.0f;
  active_pitch_direction = payload["pitch_active"] | 0.0f;
  active_roll_direction = payload["roll_active"] | 0.0f;
  active_body_yaw_direction = payload["body_yaw_active"] | 0.0f;
}

void processCenteringIntent(JsonObjectConst payload) {
  if (app_is_enabled_by_switch) return; // Ignore Python GUI centering if mobile active
  active_centering_xy = payload["center_xy_active"] | false;
  active_centering_orientation = payload["center_orientation_active"] | false;
}

void processGaitCommand(JsonObjectConst payload) {
  // This can be set by either GUI or potentially mobile app if it had such a button
  if (payload.containsKey("walk_active")) {
    walkCycleRunning = payload["walk_active"] | false;
  }
}

void processConfigUpdate(JsonObjectConst payload) {
  if (logUdpPackets) Serial.println("Processing 'config_update'");
  if (payload.containsKey("max_speeds")) {
    JsonObjectConst speeds = payload["max_speeds"].as<JsonObjectConst>();
    cfg_max_linear_speed_cms = speeds["linear_cms"] | cfg_max_linear_speed_cms;
    cfg_max_yaw_rate_rads = speeds["yaw_rads"] | cfg_max_yaw_rate_rads;
  }
  if (payload.containsKey("pose_adjust_speeds")) {
    JsonObjectConst speeds = payload["pose_adjust_speeds"].as<JsonObjectConst>();
    cfg_pose_adjust_linear_cms = speeds["linear_cms"] | cfg_pose_adjust_linear_cms;
    cfg_pose_adjust_angular_rads = speeds["angular_rads"] | cfg_pose_adjust_angular_rads;
  }
  if (payload.containsKey("gait_params")) {
    JsonObjectConst gait = payload["gait_params"].as<JsonObjectConst>();
    walkParams.stepHeight = gait["step_height_cm"] | walkParams.stepHeight;
    walkParams.stepTime = gait["step_time_s"] | walkParams.stepTime;
    // walkCycleRunning also part of gait_params in old model, now separate in gait_command
  }
  if (payload.containsKey("base_foot_positions_walk_cm")) {
    JsonArrayConst positions = payload["base_foot_positions_walk_cm"].as<JsonArrayConst>();
    if (positions) {
      int leg_idx = 0;
      for (JsonVariantConst v : positions) {
        if (leg_idx < LEG_COUNT && v.is<JsonObjectConst>()) {
          JsonObjectConst leg_pos = v.as<JsonObjectConst>();
          baseFootPositionWalk[leg_idx].x = leg_pos["x"] | baseFootPositionWalk[leg_idx].x;
          baseFootPositionWalk[leg_idx].y = leg_pos["y"] | baseFootPositionWalk[leg_idx].y;
          baseFootPositionWalk[leg_idx].z = leg_pos["z"] | baseFootPositionWalk[leg_idx].z;
          leg_idx++;
        }
      }
    }
  }
}

void processClientSettings(JsonObjectConst payload) {
  // (Implementation from previous message, for telemetry_target and requested_telemetry)
  if (logUdpPackets) Serial.println("Processing 'client_settings'");
  if (payload.containsKey("telemetry_target")) {
    JsonObjectConst target = payload["telemetry_target"].as<JsonObjectConst>();
    if (target) {
      JsonVariantConst ip_variant = target["ip"];
      if (ip_variant.is<const char*>()) {
        const char* ip_str = ip_variant.as<const char*>();
        if (telemetry_destination_ip.fromString(ip_str)) {
          telemetry_destination_port = target["port"] | 0;
          telemetry_target_known = (telemetry_destination_port != 0);
          Serial.printf("Telemetry target: %s:%u\n", ip_str, telemetry_destination_port);
        } else { Serial.println("[WARN] Invalid telemetry IP"); telemetry_target_known = false; }
      } else { Serial.println("[WARN] Telemetry IP not string"); }
    }
  }
  if (payload.containsKey("requested_telemetry")) {
    JsonObjectConst req = payload["requested_telemetry"].as<JsonObjectConst>();
    if (req) {
      if (req["battery"].is<JsonObjectConst>()) {
        sub_battery.enabled = req["battery"]["enabled"] | false;
        sub_battery.interval_ms = req["battery"]["interval_ms"] | 1000;
        sub_battery.last_sent_ms = 0; // Force send on next cycle if enabled
      }
      if (req["robot_status"].is<JsonObjectConst>()) {
        sub_robot_status.enabled = req["robot_status"]["enabled"] | false;
        sub_robot_status.interval_ms = req["robot_status"]["interval_ms"] | 5000;
        sub_robot_status.last_sent_ms = 0;
      }
      if (req["robot_state_actual"].is<JsonObjectConst>()) {
        sub_robot_state_actual.enabled = req["robot_state_actual"]["enabled"] | false;
        sub_robot_state_actual.interval_ms = req["robot_state_actual"]["interval_ms"] | 200;
        sub_robot_state_actual.last_sent_ms = 0;
      }
      if (req["debug_foot_pos"].is<JsonObjectConst>()) {
          sub_debug_foot_pos.enabled = req["debug_foot_pos"]["enabled"] | false;
          sub_debug_foot_pos.interval_ms = req["debug_foot_pos"]["interval_ms"] | 200;
          sub_debug_foot_pos.last_sent_ms = 0;
          if (logUdpPackets) Serial.printf("Subscription: debug_foot_pos enabled: %s, interval: %u\n", sub_debug_foot_pos.enabled ? "true" : "false", sub_debug_foot_pos.interval_ms);
      }
    }
  }
}

void processSystemCommand(JsonObjectConst payload) {
  if (logUdpPackets) Serial.println("Processing 'system_command'");
  const char* action = payload["action"];
  if (!action) return;

  if (strcmp(action, "reinitialize_pwm") == 0) {
    Serial.println("CMD: Re-initializing PWM drivers...");
    // Consider if servos should go to a safe pose before/after
    setupPwm(); // This should re-run .begin() and set freq
    Serial.println("PWM drivers re-initialized.");
    // Optionally send an ack/response telemetry
  }
}


// --- Mobile App Input Processing ---
void parseAndStoreMobileAppInput(const JsonDocument& doc) {
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
    if (logUdpPackets) Serial.printf("Mobile App Control Switch: %s\n", app_is_enabled_by_switch ? "ON" : "OFF");
    if (!app_is_enabled_by_switch) { // When mobile control is turned OFF, reset its intents
        active_target_vx_factor = 0.0f; active_target_vy_factor = 0.0f; active_target_yaw_factor = 0.0f;
        active_offset_x_direction = 0.0f; /* ... all active_offset/pitch/roll/body_yaw_direction to 0 ...*/
        active_centering_xy = false; active_centering_orientation = false;
        // Python GUI will need to send fresh intents if it was previously controlling
    }
  } else if (strcmp(type, "BUTTON") == 0) {
    bool pressed = (doc["state"].as<const char*>()[0] == 'P'); // Simple "PRESS" check
    if (strcmp(id, "center_xy") == 0) app_hold_center_xy = pressed;
    else if (strcmp(id, "center_rot") == 0) app_hold_center_rot = pressed;
    else if (strcmp(id, "yaw+") == 0) app_hold_yaw_plus = pressed;
    else if (strcmp(id, "yaw-") == 0) app_hold_yaw_minus = pressed;
    else if (strcmp(id, "up") == 0) app_hold_body_z_up = pressed;
    else if (strcmp(id, "down") == 0) app_hold_body_z_down = pressed; // Was "button"
  } else if (strcmp(type, "DPAD") == 0) {
    bool pressed = (doc["state"].as<const char*>()[0] == 'P');
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

void translateMobileInputsToActiveIntents() {
  // This function is called only if app_is_enabled_by_switch is true.
  // It translates the app_raw/app_hold states into the active_intent variables.

  // Locomotion
  active_target_vy_factor = clampf(app_raw_joystick_y, -1.0f, 1.0f);
  active_target_vx_factor = clampf(app_raw_joystick_x, -1.0f, 1.0f); // Assuming joystick X is for strafe

  if (fabs(app_raw_steering_angle_deg) > 0.1f) { // Prioritize steering wheel
    active_target_yaw_factor = clampf(app_raw_steering_angle_deg / 90.0f, -1.0f, 1.0f); // Normalize
  } else if (app_hold_yaw_plus) {
    active_target_yaw_factor = 1.0f;
  } else if (app_hold_yaw_minus) {
    active_target_yaw_factor = -1.0f;
  } else {
    active_target_yaw_factor = 0.0f;
  }

  // Pose Adjustments (DPAD_XY -> active_offset_*_direction)
  active_offset_y_direction = (app_hold_dpad_xy_up ? 1.0f : 0.0f) + (app_hold_dpad_xy_down ? -1.0f : 0.0f);
  active_offset_x_direction = (app_hold_dpad_xy_left ? -1.0f : 0.0f) + (app_hold_dpad_xy_right ? 1.0f : 0.0f); // GUI: Left= -X, Right=+X
  active_offset_z_direction = (app_hold_body_z_up ? 1.0f : 0.0f) + (app_hold_body_z_down ? -1.0f : 0.0f);

  // Pose Orientation (DPAD_ROT -> active_pitch/roll_direction)
  active_pitch_direction = (app_hold_dpad_rot_pitch_up ? 1.0f : 0.0f) + (app_hold_dpad_rot_pitch_down ? -1.0f : 0.0f); // App UP = Pitch Nose Up
  active_roll_direction = (app_hold_dpad_rot_roll_left ? -1.0f : 0.0f) + (app_hold_dpad_rot_roll_right ? 1.0f : 0.0f); // App Left = Roll Left
  // Note: No direct body_yaw_direction from mobile app DPAD, handled by steering/yaw buttons.

  // Centering Intents
  active_centering_xy = app_hold_center_xy;
  active_centering_orientation = app_hold_center_rot;

  // Walk active: Mobile app doesn't have a direct walk toggle in the logs, so this is not set here.
  // It's controlled by Python GUI's gait_command or config_update.
}


// --- ESP32 State Integration (called every loop) ---
void integrateRobotState(float dt) {
  // 1. Update Target Velocities (smoothly ramp to target factors)
  float target_vx = active_target_vx_factor * cfg_max_linear_speed_cms;
  float target_vy = active_target_vy_factor * cfg_max_linear_speed_cms;
  float target_yaw = active_target_yaw_factor * cfg_max_yaw_rate_rads;

  float accel_lin = 15.0f * cfg_max_linear_speed_cms * dt; // Faster accel/decel
  float accel_ang = 15.0f * cfg_max_yaw_rate_rads * dt;

  bodyVelocity.x = clampf(bodyVelocity.x + clampf(target_vx - bodyVelocity.x, -accel_lin, accel_lin), -cfg_max_linear_speed_cms, cfg_max_linear_speed_cms);
  bodyVelocity.y = clampf(bodyVelocity.y + clampf(target_vy - bodyVelocity.y, -accel_lin, accel_lin), -cfg_max_linear_speed_cms, cfg_max_linear_speed_cms);
  bodyAngularVelocityYaw = clampf(bodyAngularVelocityYaw + clampf(target_yaw - bodyAngularVelocityYaw, -accel_ang, accel_ang), -cfg_max_yaw_rate_rads, cfg_max_yaw_rate_rads);

  // 2. Update Body Position Offset
  if (active_centering_xy) {
    float linear_centering_step = cfg_pose_adjust_linear_cms * dt;
    if (fabs(bodyPositionOffset.x) < 0.1f) bodyPositionOffset.x = 0.0f;
    else if (bodyPositionOffset.x > 0) bodyPositionOffset.x -= linear_centering_step;
    else bodyPositionOffset.x += linear_centering_step;

    if (fabs(bodyPositionOffset.y) < 0.1f) bodyPositionOffset.y = 0.0f;
    else if (bodyPositionOffset.y > 0) bodyPositionOffset.y -= linear_centering_step;
    else bodyPositionOffset.y += linear_centering_step;
    // Z centering could be added if a button for it exists
  } else {
    bodyPositionOffset.x += active_offset_x_direction * cfg_pose_adjust_linear_cms * dt;
    bodyPositionOffset.y += active_offset_y_direction * cfg_pose_adjust_linear_cms * dt;
  }
  bodyPositionOffset.z += active_offset_z_direction * cfg_pose_adjust_linear_cms * dt;
  // Add clamping for bodyPositionOffset X, Y, Z to reasonable limits

  // 3. Update Body Orientation
  if (active_centering_orientation) {
    // SLERP towards identity. Factor determines speed (e.g., 0.05 means 5% of way each frame at ~50Hz)
    // A larger factor means faster centering. (e.g. 2.0*dt would try to center in 0.5s)
    float slerp_factor = 3.0f * dt; // Adjust this for desired centering speed
    bodyOrientation = slerp(bodyOrientation, Quaternion::identity(), slerp_factor);
    if (dot(bodyOrientation, Quaternion::identity()) > 0.99999f) { // Close enough
        bodyOrientation = Quaternion::identity();
        // active_centering_orientation = false; // Or let button release handle
    }
  } else {
    Quaternion rot_delta = Quaternion::identity();
    if (fabs(active_pitch_direction) > 0.01) {
      rot_delta = rot_delta * Quaternion::from_axis_angle({1.0f, 0.0f, 0.0f}, active_pitch_direction * cfg_pose_adjust_angular_rads * dt);
    }
    if (fabs(active_roll_direction) > 0.01) {
      rot_delta = rot_delta * Quaternion::from_axis_angle({0.0f, 1.0f, 0.0f}, active_roll_direction * cfg_pose_adjust_angular_rads * dt);
    }
    if (fabs(active_body_yaw_direction) > 0.01) { // Direct body yaw (e.g. from Python U/O keys)
      rot_delta = rot_delta * Quaternion::from_axis_angle({0.0f, 0.0f, 1.0f}, active_body_yaw_direction * cfg_pose_adjust_angular_rads * dt);
    }
    bodyOrientation = bodyOrientation * rot_delta;
    bodyOrientation.normalize();
  }
}

// --- Telemetry Sending ---
void sendConfiguredTelemetry() {
  uint32_t current_ms = millis();
  StaticJsonDocument<256> simple_telemetry_doc; // For battery, robot_status

  // --- Battery Telemetry ---
  if (sub_battery.enabled && (current_ms - sub_battery.last_sent_ms >= sub_battery.interval_ms)) {
    simple_telemetry_doc.clear();
    simple_telemetry_doc["type"] = "telemetry_data";
    simple_telemetry_doc["source"] = "esp32_hexapod";
    simple_telemetry_doc["timestamp_ms"] = current_ms;
    JsonObject payload = simple_telemetry_doc.createNestedObject("payload");
    JsonObject battery_data = payload.createNestedObject("battery");
    battery_data["voltage_v"] = readBatteryVoltage();

    String json_output; serializeJson(simple_telemetry_doc, json_output);
    udp.beginPacket(telemetry_destination_ip, telemetry_destination_port);
    udp.print(json_output); udp.endPacket();
    if (logUdpPackets) { Serial.print("TX Batt Teledata: "); Serial.println(json_output); }
    sub_battery.last_sent_ms = current_ms;
  }

  // --- Robot Status Telemetry ---
  if (sub_robot_status.enabled && (current_ms - sub_robot_status.last_sent_ms >= sub_robot_status.interval_ms)) {
    simple_telemetry_doc.clear();
    simple_telemetry_doc["type"] = "telemetry_data";
    simple_telemetry_doc["source"] = "esp32_hexapod";
    simple_telemetry_doc["timestamp_ms"] = current_ms;
    JsonObject payload = simple_telemetry_doc.createNestedObject("payload");
    JsonObject status_data = payload.createNestedObject("robot_status");
    status_data["wifi_rssi_dbm"] = WiFi.RSSI();
    status_data["active_controller_hint"] = app_is_enabled_by_switch ? "mobile_app" : "python_gui";

    String json_output; serializeJson(simple_telemetry_doc, json_output);
    udp.beginPacket(telemetry_destination_ip, telemetry_destination_port);
    udp.print(json_output); udp.endPacket();
    if (logUdpPackets) { Serial.print("TX Status Teledata: "); Serial.println(json_output); }
    sub_robot_status.last_sent_ms = current_ms;
  }

  // --- Robot State Actual (and optional Debug Foot Pos) Telemetry ---
  bool send_robot_state_message_this_cycle = false;
  if (sub_robot_state_actual.enabled && (current_ms - sub_robot_state_actual.last_sent_ms >= sub_robot_state_actual.interval_ms)) {
    send_robot_state_message_this_cycle = true;
  }
  if (sub_debug_foot_pos.enabled && (current_ms - sub_debug_foot_pos.last_sent_ms >= sub_debug_foot_pos.interval_ms)) {
    send_robot_state_message_this_cycle = true;
  }

  if (send_robot_state_message_this_cycle) {
    DynamicJsonDocument telemetry_doc(1024); // Adjust size if needed, 1024 should be plenty

    telemetry_doc["type"] = "robot_state_telemetry";
    telemetry_doc["source"] = "esp32_hexapod";
    telemetry_doc["timestamp_ms"] = current_ms;
    
    JsonObject payload = telemetry_doc.createNestedObject("payload");

    // Conditionally include standard robot state
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
    }

    // Conditionally include debug_foot_pos_walk_cm (using legCycleData[i].currentPosition)
    if (sub_debug_foot_pos.enabled) {
        JsonArray foot_pos_walk_array = payload.createNestedArray("debug_foot_pos_walk_cm");
        for (int i = 0; i < LEG_COUNT; ++i) {
            JsonObject leg_pos = foot_pos_walk_array.createNestedObject();
            // legCycleData is declared in walkcycle.h and defined in walkcycle.cpp
            // We need to make sure it's accessible here.
            // It's better to have walkcycle.h declare "extern LegCycleData legCycleData[LEG_COUNT];"
            // and walkcycle.cpp define "LegCycleData legCycleData[LEG_COUNT];"
            leg_pos["x"] = round(legCycleData[i].currentPosition.x * 100.0f) / 100.0f;
            leg_pos["y"] = round(legCycleData[i].currentPosition.y * 100.0f) / 100.0f;
            leg_pos["z"] = round(legCycleData[i].currentPosition.z * 100.0f) / 100.0f;
        }
    }

    // Send the packet if payload is not empty
    if (!payload.isNull() && payload.size() > 0) {
        String json_output; 
        size_t actual_size = measureJson(telemetry_doc);
        if (actual_size > telemetry_doc.capacity()) {
            Serial.printf("[ERROR RC] Robot state telemetry JSON too large! Actual: %u, Capacity: %u\n", actual_size, telemetry_doc.capacity());
        } else {
            serializeJson(telemetry_doc, json_output);
            udp.beginPacket(telemetry_destination_ip, telemetry_destination_port);
            udp.print(json_output); 
            udp.endPacket();
            if (logUdpPackets && (millis() % 1000 < 50)) { // Log sample once a second
                Serial.print("TX State Teledata (Sample): "); Serial.println(json_output);
            }
        }
    } else {
        if (logUdpPackets) Serial.println("[INFO RC] robot_state_telemetry: No data to send this cycle (all relevant subscriptions off or not due).");
    }

    // Update last_sent_ms for subscriptions that triggered the send
    if (sub_robot_state_actual.enabled && (current_ms - sub_robot_state_actual.last_sent_ms >= sub_robot_state_actual.interval_ms) ) {
         sub_robot_state_actual.last_sent_ms = current_ms;
    }
    if (sub_debug_foot_pos.enabled && (current_ms - sub_debug_foot_pos.last_sent_ms >= sub_debug_foot_pos.interval_ms) ) {
         sub_debug_foot_pos.last_sent_ms = current_ms;
    }
  }
}