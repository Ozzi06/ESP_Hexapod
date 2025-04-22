#ifndef KINEMATICS_DEBUG_H
#define KINEMATICS_DEBUG_H

#include <math.h>       // For fmodf, sinf, cosf etc.
#include <stdint.h>
#include "Vec3.h"
#include "quat.h"       // Assumes Euler conversion helpers might be added here
#include "robot_spec.h" // For globals, constants, leg_names
#include "walkcycle.h"  // For WalkParams, bell_curve_lift, quintic_interpolate_pos etc.
#include "body_transform.h" // For transformWalkFrameToLegFrame (or its logic)
#include "ik.h"         // For calculateIK
#include "utils.h"      // For servo commands, clampf, etc.

// --- Forward Declarations ---
void print_kinematics_debug_menu();
void display_kinematic_trace_horizontal();
void trigger_calculation(bool apply_to_servos);
void display_global_state();
bool parse_float_arg(const String& arg, float& value); // Helper for parsing
bool process_kinematics_debug_commands();
Vec3 quat_to_euler_rpy_degrees(const Quaternion& q);
Quaternion quat_from_euler_rpy_degrees(const Vec3& rpy_deg);

// --- Local State for Debug Mode ---
uint8_t debug_selected_leg = 0;
const char* debug_selected_leg_name = "Unknown"; // Will be set in setup

// Input Source Control
bool debug_use_globals = true; // True = use main global vars, False = use debug_ vars below

// Local copies/overrides for debug state
Vec3 debug_bodyPositionOffset;
Quaternion debug_bodyOrientation;
Vec3 debug_bodyVelocity;
WalkParams debug_walkParams;
float debug_globalPhase = 0.0f;

// Walk Cycle Playback State
enum WalkPlaybackState { PAUSED, PLAYING, STEPPING };
WalkPlaybackState debug_walk_playback_state = PAUSED;
float debug_walk_step_increment = 0.01f;
float debug_walk_playback_speed = 1.0f;
unsigned long debug_last_update_micros = 0;

// Storage for last calculation results (for display)
Vec3 last_P_foot_walk{NAN, NAN, NAN};
Vec3 last_P_foot_body{NAN, NAN, NAN};
Vec3 last_V_leg_to_foot_body{NAN, NAN, NAN};
Vec3 last_P_foot_leg_ik_input{NAN, NAN, NAN};
Vec3 last_angles_rad{NAN, NAN, NAN};
Vec3 last_angles_deg{NAN, NAN, NAN};
uint16_t last_pulses[3] = {0, 0, 0};

// --- Setup Function ---
void setup_kinematics_debug() {
    Serial.println("\n===== Kinematics Debugger =====");

    // Initialize local state from globals
    debug_bodyPositionOffset = bodyPositionOffset;
    debug_bodyOrientation = bodyOrientation;
    debug_bodyVelocity = bodyVelocity;
    debug_walkParams = walkParams; // Struct copy
    debug_globalPhase = globalPhase;

    debug_use_globals = true; // Default to using global state
    debug_walk_playback_state = PAUSED;
    debug_walk_playback_speed = 1.0f;
    debug_walk_step_increment = 0.01f;
    debug_last_update_micros = 0; // Reset timer

    // Initialize leg selection
    debug_selected_leg = 0;
    if (leg_names != nullptr && debug_selected_leg < LEG_COUNT) { // Check if leg_names is defined
         debug_selected_leg_name = leg_names[debug_selected_leg];
    } else {
        debug_selected_leg_name = "Leg 0 (Name N/A)";
    }

    // Clear previous calculation results
    last_P_foot_walk = {NAN, NAN, NAN};
    last_P_foot_body = {NAN, NAN, NAN};
    last_V_leg_to_foot_body = {NAN, NAN, NAN};
    last_P_foot_leg_ik_input = {NAN, NAN, NAN};
    last_angles_rad = {NAN, NAN, NAN};
    last_angles_deg = {NAN, NAN, NAN};
    last_pulses[0] = last_pulses[1] = last_pulses[2] = 0;


    print_kinematics_debug_menu();
    Serial.println("--- Initial State ---");
    display_global_state(); // Show initial global state
}

// --- Main Update Function for this Mode ---
// Returns true to continue, false to exit mode
bool update_kinematics_debug() {
    unsigned long now_micros = micros();
    // Prevent initial jump or large gaps in dt
    if (debug_last_update_micros == 0 || now_micros < debug_last_update_micros) {
         debug_last_update_micros = now_micros;
    }
    float dt = (now_micros - debug_last_update_micros) / 1000000.0f;
    debug_last_update_micros = now_micros;

    // Process serial commands first
    if (!process_kinematics_debug_commands()) {
        return false; // Exit command received
    }

    // Handle walk cycle playback state
    if (debug_walk_playback_state == PLAYING) {
        // Determine which frequency to use
        float freq_to_use = debug_use_globals ? walkParams.stepFrequency : debug_walkParams.stepFrequency;

        // Advance phase
        debug_globalPhase = fmodf(debug_globalPhase + (freq_to_use * dt * debug_walk_playback_speed), 1.0f);
        if (debug_globalPhase < 0.0f) debug_globalPhase += 1.0f; // Ensure positive phase

        // Trigger calculation and display (don't move servos)
        trigger_calculation(false);
    } else if (debug_walk_playback_state == STEPPING) {
        // Phase was already adjusted by step command
        // Trigger calculation and display for this single step
        trigger_calculation(false);
        // Reset state to paused after processing the step
        debug_walk_playback_state = PAUSED;
        Serial.printf("\n(Stepping complete, now Paused at Phase: %.3f)\n> ", debug_globalPhase);
    }

    return true; // Continue running
}

// --- Command Processing ---
// Returns true to continue, false to exit
bool process_kinematics_debug_commands() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0) return true; // Ignore empty lines

        char command = toupper(input.charAt(0));
        String arg = "";
        if (input.length() > 1) {
            arg = input.substring(1);
            arg.trim(); // Trim argument whitespace
        }

        // --- Parse and Handle Commands ---
        switch (command) {
            case 'H': print_kinematics_debug_menu(); break;
            case 'X': Serial.println("Exiting Kinematics Debugger..."); return false;

            // --- Control ---
            case 'L': {
                uint8_t leg_idx = arg.toInt();
                if (leg_idx < LEG_COUNT) {
                    debug_selected_leg = leg_idx;
                    if (leg_names != nullptr) {
                        debug_selected_leg_name = leg_names[debug_selected_leg];
                    } else {
                        char name_buf[20];
                        sprintf(name_buf, "Leg %d (Name N/A)", debug_selected_leg);
                        // Need a static buffer or manage memory if assigning directly
                        // For simplicity, maybe just print it directly
                        debug_selected_leg_name = "Name N/A"; // Reset placeholder
                    }
                    Serial.printf("Selected Leg %d (%s)\n", debug_selected_leg,
                                   (leg_names != nullptr) ? debug_selected_leg_name : "Name N/A");
                } else {
                    Serial.println("Invalid leg index.");
                }
                break;
            }
            case 'S': { // Sub-commands G or D
                if (arg.length() > 0) {
                    char sub_cmd = toupper(arg.charAt(0));
                    if (sub_cmd == 'G') {
                        debug_use_globals = true;
                        Serial.println("Input Source: Using GLOBAL state variables.");
                    } else if (sub_cmd == 'D') {
                        debug_use_globals = false;
                        Serial.println("Input Source: Using DEBUG state variables.");
                    } else {
                        Serial.println("Invalid source command. Use SG or SD.");
                    }
                } else {
                     Serial.println("Missing source specifier. Use SG or SD.");
                }
                break;
            }
            case 'C': trigger_calculation(false); break; // Calculate only
            case 'A': {
                Serial.println("\n*** WARNING: Applying calculated pulses to physical servos! ***");
                trigger_calculation(true); // Calculate AND Apply
                break;
            }

            // --- Display ---
            case 'D': display_kinematic_trace_horizontal(); break;
            case 'G': display_global_state(); break; // Renamed sub-command was confusing

            // --- Walk Cycle Playback ---
            case 'P': { // Play or sub-commands S/I
                 if (arg.length() > 0) {
                     char sub_cmd = toupper(arg.charAt(0));
                     String sub_arg = arg.substring(1);
                     float val;
                     if (sub_cmd == 'S' && parse_float_arg(sub_arg, val)) {
                         debug_walk_playback_speed = max(0.1f, val); // Prevent zero/negative speed
                         Serial.printf("Playback Speed set to: %.2f x\n", debug_walk_playback_speed);
                     } else if (sub_cmd == 'I' && parse_float_arg(sub_arg, val)) {
                         debug_walk_step_increment = max(0.001f, abs(val)); // Prevent zero/negative increment
                         Serial.printf("Step Increment set to: %.3f\n", debug_walk_step_increment);
                     } else {
                         Serial.println("Invalid playback subcommand/value. Use Ps[speed] or Pi[increment].");
                     }
                 } else {
                     // No sub-command, just PLAY
                     if (debug_walk_playback_state != PLAYING) {
                         debug_walk_playback_state = PLAYING;
                         debug_last_update_micros = 0; // Reset timer for dt calc
                         Serial.println("Walk Cycle Playback: PLAYING");
                     }
                 }
                 break;
            }
            case ' ': { // Space bar often used for pause
                if (debug_walk_playback_state != PAUSED) {
                    debug_walk_playback_state = PAUSED;
                    Serial.println("\nWalk Cycle Playback: PAUSED");
                }
                 break;
            }
            case '.': { // Step Forward
                 debug_globalPhase = fmodf(debug_globalPhase + debug_walk_step_increment, 1.0f);
                 if (debug_globalPhase < 0.0f) debug_globalPhase += 1.0f;
                 debug_walk_playback_state = STEPPING;
                 Serial.printf("Stepping Forward to Phase: %.3f...\n", debug_globalPhase);
                 // Calculation happens in update loop
                 break;
            }
            case ',': { // Step Backward
                 debug_globalPhase = fmodf(debug_globalPhase - debug_walk_step_increment, 1.0f);
                 if (debug_globalPhase < 0.0f) debug_globalPhase += 1.0f; // Handle negative wrap
                 debug_walk_playback_state = STEPPING;
                 Serial.printf("Stepping Backward to Phase: %.3f...\n", debug_globalPhase);
                 // Calculation happens in update loop
                 break;
            }

            // --- Set DEBUG Values --- (Only affect calculations if SD is active)
            case 'O': { // Offset sub-commands X, Y, Z, P
                if (arg.length() > 0) {
                    char sub_cmd = toupper(arg.charAt(0));
                    String sub_arg = arg.substring(1);
                    float val;
                    if (sub_cmd == 'P') {
                        debug_bodyPositionOffset = {0, 0, 15.0f}; // Example reset height
                        Serial.printf("Debug Offset Reset to {%.1f, %.1f, %.1f}\n", debug_bodyPositionOffset.x, debug_bodyPositionOffset.y, debug_bodyPositionOffset.z);
                        debug_use_globals = false; // Setting implies switching to debug
                    } else if (parse_float_arg(sub_arg, val)) {
                        if (sub_cmd == 'X') debug_bodyPositionOffset.x = val;
                        else if (sub_cmd == 'Y') debug_bodyPositionOffset.y = val;
                        else if (sub_cmd == 'Z') debug_bodyPositionOffset.z = val;
                        else { Serial.println("Invalid Offset axis. Use Ox, Oy, Oz, or OP."); break; }
                        Serial.printf("Debug Offset %c set to %.2f\n", sub_cmd, val);
                        debug_use_globals = false; // Setting implies switching to debug
                    } else { Serial.println("Invalid Offset value."); }
                } else { Serial.println("Missing Offset axis/command."); }
                break;
            }
            case 'R': { // Orientation sub-commands R, P, Y, O
                if (arg.length() > 0) {
                     char sub_cmd = toupper(arg.charAt(0));
                     String sub_arg = arg.substring(1);
                     float val;
                     if (sub_cmd == 'O') {
                         debug_bodyOrientation = Quaternion::identity();
                         Serial.println("Debug Orientation Reset to Identity (0,0,0 RPY).");
                         debug_use_globals = false; // Setting implies switching to debug
                     } else if (parse_float_arg(sub_arg, val)) {
                         // Get current Euler angles, modify one, convert back
                         // Assumes quat.h has to_euler_rpy_degrees and from_euler_rpy_degrees
                         Vec3 current_rpy = quat_to_euler_rpy_degrees(debug_bodyOrientation); // Need this func
                         if (sub_cmd == 'R') current_rpy.x = val; // Assuming x=roll
                         else if (sub_cmd == 'P') current_rpy.y = val; // Assuming y=pitch
                         else if (sub_cmd == 'Y') current_rpy.z = val; // Assuming z=yaw
                         else { Serial.println("Invalid Orientation axis. Use Rr, Rp, Ry, or RO."); break; }

                         debug_bodyOrientation = quat_from_euler_rpy_degrees(current_rpy); // Need this func
                         Serial.printf("Debug Orientation %c set to %.2f deg. New RPY ~ {%.1f, %.1f, %.1f}\n",
                                       sub_cmd, val, current_rpy.x, current_rpy.y, current_rpy.z);
                         debug_use_globals = false; // Setting implies switching to debug
                     } else { Serial.println("Invalid Orientation value."); }
                } else { Serial.println("Missing Orientation axis/command."); }
                break;
            }
             case 'V': { // Velocity sub-commands X, Y, Z
                if (arg.length() > 0) {
                    char sub_cmd = toupper(arg.charAt(0));
                    String sub_arg = arg.substring(1);
                    float val;
                    if (sub_cmd == 'Z') {
                        debug_bodyVelocity = {0, 0, 0};
                        Serial.println("Debug Velocity Zeroed.");
                        debug_use_globals = false;
                    } else if (parse_float_arg(sub_arg, val)) {
                        if (sub_cmd == 'X') debug_bodyVelocity.x = val;
                        else if (sub_cmd == 'Y') debug_bodyVelocity.y = val;
                        //else if (sub_cmd == 'Z') debug_bodyVelocity.z = val; // Z velocity might not be used
                        else { Serial.println("Invalid Velocity axis. Use Vx, Vy, or VZ."); break; }
                        Serial.printf("Debug Velocity %c set to %.2f\n", sub_cmd, val);
                        debug_use_globals = false;
                    } else { Serial.println("Invalid Velocity value."); }
                } else { Serial.println("Missing Velocity axis/command."); }
                break;
            }
             case 'W': { // Walk Param sub-commands H, F, D, P
                if (arg.length() > 0) {
                    char sub_cmd = toupper(arg.charAt(0));
                    String sub_arg = arg.substring(1);
                    float val;
                    if (parse_float_arg(sub_arg, val)) {
                        if (sub_cmd == 'H') debug_walkParams.stepHeight = val;
                        else if (sub_cmd == 'F') debug_walkParams.stepFrequency = max(0.0f, val);
                        else if (sub_cmd == 'D') debug_walkParams.dutyFactor = clampf(val, 0.01f, 0.99f);
                        else if (sub_cmd == 'P') debug_globalPhase = fmodf(val, 1.0f);
                         else { Serial.println("Invalid Walk Param. Use Wh, Wf, Wd, or WP."); break; }

                        Serial.printf("Debug Walk Param %c set to %.3f\n", sub_cmd, val);
                         if (sub_cmd == 'P') Serial.printf("Debug Global Phase set to %.3f\n", debug_globalPhase);

                        debug_use_globals = false;
                    } else { Serial.println("Invalid Walk Param value."); }
                } else { Serial.println("Missing Walk Param command."); }
                break;
            }

            default:
                Serial.print("Unknown command: '");
                Serial.print(command);
                Serial.println("'. Type 'H' for help.");
                break;
        }
        Serial.print("> "); // Prompt for next command
    }
    return true; // Continue running unless 'X' was processed
}


// --- Helper Functions ---

bool parse_float_arg(const String& arg, float& value) {
    if (arg.length() == 0) return false;
    // Basic parsing, doesn't handle errors robustly but okay for debug tool
    value = arg.toFloat();
    // Check if conversion was successful (e.g., avoids issues if input is non-numeric)
    // A simple check: if the string isn't "0" or "-0" but toFloat returns 0, it might be an error.
    // More robust parsing might be needed if this causes issues.
    if (value == 0.0f && arg != "0" && arg != "-0" && arg != "0.0" && arg != "-0.0") {
         // It's possible 0 was intended, but this catches many non-numeric inputs
         // Let's assume it's okay for now in a debug context.
    }
    return true; // Assume success for simplicity here
}


// Placeholder Euler conversion functions - IMPLEMENT THESE IN QUAT_H/CPP
// Or use a library if available. These are simplified examples.
Vec3 quat_to_euler_rpy_degrees(const Quaternion& q) {
    // WARNING: This is a simplified conversion and subject to gimbal lock.
    // A robust implementation is more complex. Replace with actual implementation.
    Serial.println("[WARN] Using simplified placeholder for quat_to_euler_rpy_degrees!");
    float roll = atan2f(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * 180.0f / M_PI;
    float pitch_arg = 2.0f*(q.w*q.y - q.z*q.x);
    pitch_arg = clampf(pitch_arg, -1.0f, 1.0f); // Clamp argument for asin
    float pitch = asinf(pitch_arg) * 180.0f / M_PI;
    float yaw = atan2f(2.0f*(q.w*q.z + q.x*q.y), 1.0f - 2.0f*(q.y*q.y + q.z*q.z)) * 180.0f / M_PI;
    return {roll, pitch, yaw};
}

Quaternion quat_from_euler_rpy_degrees(const Vec3& rpy_deg) {
    // WARNING: Placeholder. Replace with actual implementation.
     Serial.println("[WARN] Using simplified placeholder for quat_from_euler_rpy_degrees!");
    float r_rad = rpy_deg.x * M_PI / 180.0f;
    float p_rad = rpy_deg.y * M_PI / 180.0f;
    float y_rad = rpy_deg.z * M_PI / 180.0f;
    float cy = cosf(y_rad * 0.5f); float sy = sinf(y_rad * 0.5f);
    float cp = cosf(p_rad * 0.5f); float sp = sinf(p_rad * 0.5f);
    float cr = cosf(r_rad * 0.5f); float sr = sinf(r_rad * 0.5f);
    return Quaternion( cy * cp * cr + sy * sp * sr,
                       cy * cp * sr - sy * sp * cr,
                       sy * cp * sr + cy * sp * cr,
                       sy * cp * cr - cy * sp * sr );
}


// --- Core Calculation Function ---
void trigger_calculation(bool apply_to_servos) {
    // 1. Select Input Source
    const Vec3&       pos_offset_in = debug_use_globals ? bodyPositionOffset : debug_bodyPositionOffset;
    const Quaternion& orientation_in  = debug_use_globals ? bodyOrientation : debug_bodyOrientation;
    const Vec3&       velocity_in     = debug_use_globals ? bodyVelocity : debug_bodyVelocity;
    const WalkParams& walk_params_in  = debug_use_globals ? walkParams : debug_walkParams;
    // Phase always uses the debug_globalPhase for playback/stepping control
    float             phase_in        = debug_globalPhase;


    // 2. Simulate Walk Cycle Target (P_foot_walk) for the selected leg/phase
    //    This replicates the core logic from walkcycle.h for one step
    Vec3 base_pos = {0,0,0}; // Need the actual base position for this leg!
    // TODO: Get base position from robot_spec.h or initial walkcycle setup
    // For now, using a placeholder - THIS NEEDS FIXING
    // Maybe use legOriginOffset as a rough standing pos relative to body, then transform to walk frame?
    // Let's assume legCycleData[debug_selected_leg].basePosition IS the walk frame neutral pos
    base_pos = legCycleData[debug_selected_leg].basePosition; // ASSUMPTION!

    float T_cycle = (walk_params_in.stepFrequency > 1e-6) ? 1.0f / walk_params_in.stepFrequency : 1.0f;
    float duty_factor = clampf(walk_params_in.dutyFactor, 0.01f, 0.99f);
    float T_stance = duty_factor * T_cycle;
    float T_swing = T_cycle - T_stance;

    if (phase_in >= duty_factor) { // In Swing Phase
        float stepPhase = (phase_in - duty_factor) / (1.0f - duty_factor); // Normalize 0..1
        float timeInSwing = stepPhase * T_swing;

        // Calculate target touchdown in Walk Frame
        float predictionTime = T_stance * 0.5f; // How far ahead to place the foot
        Vec3 predictedBodyMovement = velocity_in * predictionTime;
        Vec3 targetTouchdownPos = base_pos + predictedBodyMovement; // Target landing pos in Walk Frame

        // Z position (Vertical lift in Walk Frame)
        float lift_curve = bell_curve_lift(stepPhase);
        last_P_foot_walk.z = base_pos.z + walk_params_in.stepHeight * lift_curve;

        // XY position (Interpolation in Walk Frame)
        // Need swing start position! Approximation: Use base_pos projected backward
        Vec3 swingStartPositionApprox = base_pos - velocity_in * (T_stance * 0.5f);
        Vec3 swingVelStart = {0,0,0}; // Simple start/end velocity for debug
        Vec3 swingVelEnd   = {0,0,0};

        last_P_foot_walk.x = quintic_interpolate_pos(
            swingStartPositionApprox.x, targetTouchdownPos.x,
            swingVelStart.x, swingVelEnd.x, T_swing, timeInSwing);

        last_P_foot_walk.y = quintic_interpolate_pos(
            swingStartPositionApprox.y, targetTouchdownPos.y,
            swingVelStart.y, swingVelEnd.y, T_swing, timeInSwing);

    } else { // In Stance Phase
        // Foot should be moving backward relative to where it landed to stay fixed in Walk Frame
        // Position = touchdown_pos - velocity * time_in_stance
        // Approx touchdown_pos = base_pos + velocity * T_stance * 0.5f
        // time_in_stance = phase_in * T_cycle = phase_in / freq
        // Position = base_pos + vel*(T_s/2) - vel*(phase/f)
        // Position = base_pos + vel * (T_s/2 - phase * T_c)
         float time_in_stance = phase_in * T_cycle;
         Vec3 stanceStartPosApprox = base_pos + velocity_in * (T_stance * 0.5f);
         last_P_foot_walk = stanceStartPosApprox - velocity_in * time_in_stance;

         // Z position stays at base during stance
         last_P_foot_walk.z = base_pos.z;
    }


    // 3. Perform Body Transform (Replicating steps to store intermediates)
    //    Input: P_foot_walk (last_P_foot_walk), body pose (pos_offset_in, orientation_in)
    //           leg geometry (legOriginOffset[idx], legMountingAngle[idx])
    //    Outputs: P_foot_body, V_leg_to_foot_body, P_foot_leg_ik_input
    const Vec3& P_body_walk = pos_offset_in;
    const Quaternion& Q_body_walk = orientation_in;
    Quaternion Q_body_walk_inv = Q_body_walk.conjugate();

    Vec3 V_body_to_foot_walk = last_P_foot_walk - P_body_walk;
    last_P_foot_body = rotate_vector_by_quaternion(V_body_to_foot_walk, Q_body_walk_inv); // Store intermediate

    const Vec3& legOrigin_body = legOriginOffset[debug_selected_leg];
    last_V_leg_to_foot_body = last_P_foot_body - legOrigin_body; // Store intermediate

    const float mountAngle = legMountingAngle[debug_selected_leg];
    Quaternion Q_mount_inv = Quaternion::from_axis_angle({0.0f, 0.0f, 1.0f}, -mountAngle);
    last_P_foot_leg_ik_input = rotate_vector_by_quaternion(last_V_leg_to_foot_body, Q_mount_inv); // Store final IK input

    // 4. Perform Inverse Kinematics
    //    Input: P_foot_leg_ik_input (last_P_foot_leg_ik_input)
    //    Output: Angles (Coxa, Femur, Tibia)
    float c_rad=NAN, f_rad=NAN, t_rad=NAN;
    bool ik_success = calculateIK(debug_selected_leg,
                                  last_P_foot_leg_ik_input.x,
                                  last_P_foot_leg_ik_input.y,
                                  last_P_foot_leg_ik_input.z,
                                  c_rad, f_rad, t_rad);

    if (ik_success) {
        last_angles_rad = {c_rad, f_rad, t_rad};
        last_angles_deg = {c_rad * 180.0f / M_PI, f_rad * 180.0f / M_PI, t_rad * 180.0f / M_PI};

        // 5. Calculate Servo Pulses (No mirroring needed based on prior discussion)
        last_pulses[0] = get_pulse_from_angle_radians(c_rad); // Coxa
        last_pulses[1] = get_pulse_from_angle_radians(f_rad); // Femur
        last_pulses[2] = get_pulse_from_angle_radians(t_rad); // Tibia

    } else {
        // Handle IK failure - store NANs or specific error codes
        last_angles_rad = {NAN, NAN, NAN};
        last_angles_deg = {NAN, NAN, NAN};
        last_pulses[0] = last_pulses[1] = last_pulses[2] = 0; // Or a specific error pulse?
         Serial.println("[Error] IK Calculation Failed for this state!");
    }

    // 6. Display Results
    display_kinematic_trace_horizontal();

    // 7. Apply to Servos (if requested and IK succeeded)
    if (apply_to_servos && ik_success) {
        // Get servo indices from robot_spec.h
        const uint8_t coxa_servo_idx  = LEG_SERVOS[debug_selected_leg][0];
        const uint8_t femur_servo_idx = LEG_SERVOS[debug_selected_leg][1];
        const uint8_t tibia_servo_idx = LEG_SERVOS[debug_selected_leg][2];

        Serial.printf("  Applying Pulses -> C:%d(%d) F:%d(%d) T:%d(%d)\n",
            coxa_servo_idx, last_pulses[0],
            femur_servo_idx, last_pulses[1],
            tibia_servo_idx, last_pulses[2]);

        setAnglePulse(coxa_servo_idx, last_pulses[0]);
        setAnglePulse(femur_servo_idx, last_pulses[1]);
        setAnglePulse(tibia_servo_idx, last_pulses[2]);
    } else if (apply_to_servos && !ik_success) {
         Serial.println("  Skipping servo application due to IK failure.");
    }
}


// --- Display Functions ---

void display_kinematic_trace_horizontal() {
    char line_buf[200]; // Buffer for formatted lines (adjust size as needed)
    const char* leg_name_to_print = (leg_names != nullptr) ? debug_selected_leg_name : "N/A";

    // Line 1: Leg Info, Input Source, Playback State, Current Phase
    const char* input_src_str = debug_use_globals ? "GLOBAL" : "DEBUG ";
    const char* play_state_str;
    switch(debug_walk_playback_state) {
        case PLAYING:  play_state_str = "Playing"; break;
        case PAUSED:   play_state_str = "Paused "; break;
        case STEPPING: play_state_str = "Stepped"; break;
        default:       play_state_str = "Unknown"; break;
    }
    sprintf(line_buf, "Leg: %d-%-12s| Input: %s | Walk: %s @ %.1fx | Phase: %.3f",
            debug_selected_leg, leg_name_to_print, input_src_str,
            play_state_str, debug_walk_playback_speed, debug_globalPhase);
    Serial.println(line_buf);

    // Inputs Used (Select based on flag)
    const Vec3&       pos_offset_in = debug_use_globals ? bodyPositionOffset : debug_bodyPositionOffset;
    const Quaternion& orientation_in  = debug_use_globals ? bodyOrientation : debug_bodyOrientation;
    const Vec3&       velocity_in     = debug_use_globals ? bodyVelocity : debug_bodyVelocity;
    const WalkParams& walk_params_in  = debug_use_globals ? walkParams : debug_walkParams;
    Vec3 rpy_deg_in = quat_to_euler_rpy_degrees(orientation_in); // Convert for display

    // Line 2: Body Pose Inputs
    sprintf(line_buf, " Body Pose In: Offset{X:%5.1f Y:%5.1f Z:%5.1f} Orient(RPY){R:%5.1f P:%5.1f Y:%5.1f}",
            pos_offset_in.x, pos_offset_in.y, pos_offset_in.z,
            rpy_deg_in.x, rpy_deg_in.y, rpy_deg_in.z);
    Serial.println(line_buf);

    // Line 3: Walk Parameters & Velocity Inputs
    sprintf(line_buf, " Walk Params In: Height:%4.1f Freq:%.2f Duty:%.2f | Velocity In: {X:%5.1f Y:%5.1f Z:%5.1f}",
            walk_params_in.stepHeight, walk_params_in.stepFrequency, walk_params_in.dutyFactor,
            velocity_in.x, velocity_in.y, velocity_in.z);
    Serial.println(line_buf);

    Serial.println("--- Calculation Trace ---");

    // Line 4: Calculated Walk Cycle Target
    sprintf(line_buf, " => Walk Target (Walk Frame):   P{X:%5.1f Y:%5.1f Z:%5.1f}",
            last_P_foot_walk.x, last_P_foot_walk.y, last_P_foot_walk.z);
    Serial.println(line_buf);

    // Line 5: Intermediate Transformation Results
    sprintf(line_buf, " => Transform (Body Frame):     P_Body{X:%5.1f Y:%5.1f Z:%5.1f} | V_Leg{X:%5.1f Y:%5.1f Z:%5.1f}",
            last_P_foot_body.x, last_P_foot_body.y, last_P_foot_body.z,
            last_V_leg_to_foot_body.x, last_V_leg_to_foot_body.y, last_V_leg_to_foot_body.z);
    Serial.println(line_buf);

    // Line 6: Final IK Input
    sprintf(line_buf, " => IK Input (Leg Local Frame): P_IK{X:%5.1f Y:%5.1f Z:%5.1f}",
            last_P_foot_leg_ik_input.x, last_P_foot_leg_ik_input.y, last_P_foot_leg_ik_input.z);
    Serial.println(line_buf);

    // Line 7: Final IK Angles (Deg) and Servo Pulses
    sprintf(line_buf, " => IK Angles(Deg)/Pulses:      C{A:%5.1f P:%4d} F{A:%5.1f P:%4d} T{A:%5.1f P:%4d}",
            last_angles_deg.x, last_pulses[0],
            last_angles_deg.y, last_pulses[1],
            last_angles_deg.z, last_pulses[2]);
    Serial.println(line_buf);
    Serial.println("-----------------------");

}

void display_global_state() {
    Serial.println("--- Current GLOBAL State ---");
    char buf[100];
    Vec3 global_rpy = quat_to_euler_rpy_degrees(bodyOrientation);
    sprintf(buf, " Body Offset: {X:%.2f Y:%.2f Z:%.2f}", bodyPositionOffset.x, bodyPositionOffset.y, bodyPositionOffset.z);
    Serial.println(buf);
    sprintf(buf, " Body Orient: {R:%.1f P:%.1f Y:%.1f} (Deg RPY)", global_rpy.x, global_rpy.y, global_rpy.z);
    Serial.println(buf);
    sprintf(buf, " Body Velocity: {X:%.2f Y:%.2f Z:%.2f}", bodyVelocity.x, bodyVelocity.y, bodyVelocity.z);
    Serial.println(buf);
    sprintf(buf, " Walk Params: Height:%.2f Freq:%.2f Duty:%.2f", walkParams.stepHeight, walkParams.stepFrequency, walkParams.dutyFactor);
    Serial.println(buf);
    sprintf(buf, " Global Phase: %.3f", globalPhase);
    Serial.println(buf);
    Serial.println("--------------------------");
}


void print_kinematics_debug_menu() {
    char header_buf[100];
    const char* leg_name_to_print = (leg_names != nullptr) ? debug_selected_leg_name : "N/A";
    const char* input_src_str = debug_use_globals ? "GLOBAL" : "DEBUG";
    const char* play_state_str;
    switch(debug_walk_playback_state) {
        case PLAYING:  play_state_str = "Playing"; break;
        case PAUSED:   play_state_str = "Paused "; break;
        case STEPPING: play_state_str = "Stepped"; break;
        default:       play_state_str = "Unknown"; break;
    }
    sprintf(header_buf, "(Leg: %d-%s) (Input: %s) (Walk: %s @ %.1fx)",
            debug_selected_leg, leg_name_to_print, input_src_str, play_state_str, debug_walk_playback_speed);

    Serial.println("\n===== Kinematics Debugger Menu =====");
    Serial.println(header_buf);
    Serial.println("--- Control ---");
    Serial.println(" L[idx] : Select Leg (0-5)");
    Serial.println(" SG     : Use GLOBAL state for calculations");
    Serial.println(" SD     : Use DEBUG state for calculations");
    Serial.println(" C      : Calculate current state & Display");
    Serial.println(" A      : Calculate, Display & APPLY pulses to selected leg servos!");
    Serial.println("--- Display ---");
    Serial.println(" D      : Display last calculated kinematic trace (horizontal format)");
    Serial.println(" DG     : Display current GLOBAL state values");
    Serial.println(" H      : Print this Help Menu");
    Serial.println(" X      : Exit to Main Menu");
    Serial.println("--- Walk Cycle Playback ---");
    Serial.println(" P      : PLAY walk cycle (displays trace continuously)");
    Serial.println(" SPACE  : PAUSE walk cycle playback");
    Serial.println(" .      : Step Forward (advance phase, calc & display)");
    Serial.println(" ,      : Step Backward (decrement phase, calc & display)");
    Serial.println(" Ps[spd]: Set Playback Speed (e.g., Ps1.0, Ps0.5)");
    Serial.println(" Pi[inc]: Set step Increment (e.g., Pi0.01)");
    Serial.println("--- Set DEBUG Values (Only used if 'SD' active) ---");
    Serial.println(" Ox[val], Oy[val], Oz[val], OP     : Set/Reset Debug Body Offset");
    Serial.println(" Rr[val], Rp[val], Ry[val], RO     : Set/Reset Debug Body Orientation (RPY Deg)");
    Serial.println(" Vx[val], Vy[val], VZ              : Set/Zero Debug Body Velocity");
    Serial.println(" Wh[val], Wf[val], Wd[val]         : Set Debug Walk Params (H, F, D)");
    Serial.println(" WP[ph]                            : Set Debug Global Phase (0-1)");
    Serial.println("====================================");
    Serial.print("> "); // Prompt
}


#endif // KINEMATICS_DEBUG_H