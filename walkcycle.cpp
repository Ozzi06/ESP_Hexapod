#include "walkcycle.h"

WalkParams walkParams; // Use default values from struct definition initially

LegCycleData legCycleData[LEG_COUNT];
bool walkCycleRunning = false;

enum {
  TripodSwing1,
  TripodSwing2,
  IdleStance,
} walkState = IdleStance;

void setupWalkcycle() {
  walkCycleRunning = false; // Ensure walk cycle is stopped initially

  // Initialize leg cycle data based on default stance positions from robot_spec
  for (uint8_t i = 0; i < LEG_COUNT; i++) {

    // Initialize positions to the base positions (neutral stance)
    legCycleData[i].currentPosition = baseFootPositionWalk[i];
    legCycleData[i].swingStartPosition = baseFootPositionWalk[i];
  }
  Serial.println("Walk cycle initialized.");
  // Legs remain in their current position until walkCycleRunning=true and updateWalkCycle is called.
}

/**
 * @brief Generates a smooth vertical lift profile using a bell curve shape.
 * @param t Normalized time within the swing phase (0.0 to 1.0).
 * @return Lift factor (0.0 to 1.0), peaking at t=0.5.
 */
float bell_curve_lift(float t) {
    t = clampf(t, 0.0f, 1.0f);
    // 6th order polynomial approximation of a bell curve (t^3 * (1-t)^3)
    // Scaled by 64 to peak at 1.0 when t=0.5
    float t_minus_1 = 1.0f - t;
    return 64.0f * (t * t * t) * (t_minus_1 * t_minus_1 * t_minus_1);
}

/**
 * @brief Smoothly interpolates position using a quintic Hermite spline.
 * Calculates position at time 't' given start/end positions (p0, p1),
 * start/end velocities (v0, v1), and total duration (T).
 * Ensures position and velocity continuity at endpoints.
 *
 * @param p0 Position at t=0
 * @param p1 Position at t=T
 * @param v0 Velocity at t=0
 * @param v1 Velocity at t=T
 * @param T Total duration of the interpolation
 * @param t Current time (should be 0 <= t <= T)
 * @return Interpolated position at time t
 */
float quintic_interpolate_pos(float p0, float p1, float v0, float v1, float T, float t) {
    if (T <= 1e-9f) return p0; // Avoid division by zero if duration is negligible
    t = clampf(t, 0.0f, T);    // Clamp time within the valid range

    float tn = t / T; // Normalized time (0 to 1)
    float tn2 = tn*tn;
    float tn3 = tn2*tn;
    float tn4 = tn3*tn;
    float tn5 = tn4*tn;

    // Quintic Hermite basis functions (h00, h10, h01, h11 for position)
    // Note: These are derived for POSITION interpolation using start/end pos/vel.
    // (Coefficients for position: a0=p0, a1=v0, a2=0, a3=..., etc.)
    // Derivation leads to these basis functions:
    float h00 =  1.0f - 10.0f*tn3 + 15.0f*tn4 -  6.0f*tn5;
    float h10 =       tn -  6.0f*tn3 +  8.0f*tn4 -  3.0f*tn5; // Scaled by T in use
    float h01 =        + 10.0f*tn3 - 15.0f*tn4 +  6.0f*tn5;
    float h11 =        -  4.0f*tn3 +  7.0f*tn4 -  3.0f*tn5; // Scaled by T in use

    // Calculate interpolated position
    return h00*p0 + h10*T*v0 + h01*p1 + h11*T*v1;
}

float gaitProgress = 0.0f;

void leg_stance(float dt, uint8_t leg_idx){
  // --- STANCE ---
  // Foot is on the ground and should move backward relative to the body/Walk frame
  // to remain stationary relative to the Ground.

  LegCycleData& leg = legCycleData[leg_idx]; // Get reference to this leg's state

  // Calculate the current foot position by moving backward from the touchdown point
  // based on how long the foot has been in stance.
  leg.currentPosition = leg.currentPosition - (bodyVelocity * dt);
        
  // Calculate the counter-rotation angle for this timestep
  float delta_angle = -bodyAngularVelocityYaw * dt;
  float cos_da = cosf(delta_angle);
  float sin_da = sinf(delta_angle);
  float original_x = leg.currentPosition.x;
  float original_y = leg.currentPosition.y;
  leg.currentPosition.x = original_x * cos_da - original_y * sin_da;
  leg.currentPosition.y = original_x * sin_da + original_y * cos_da;
  // Z remains unchanged by this 2D rotation



  // **Critical:** Record the current position as the starting point for the *next* swing phase.
  // This captures where the foot actually was just before lifting off.
  leg.swingStartPosition = leg.currentPosition;
}

void leg_swing(float dt, uint8_t leg_idx){
  // --- SWING ---
  // Foot is in the air, moving from swingStartPosition to the next target touchdown position.

  LegCycleData& leg = legCycleData[leg_idx]; // Get reference to this leg's state
  
  // The time elapsed *within* the current swing phase
  float time_in_swing = gaitProgress * walkParams.stepTime;

  // The target touchdown position for the *end* of this swing phase.
  // This is where the *next* stance phase will begin. (Same calculation as stanceTouchdownPos above)
  Vec3 linearTargetTouchdownPos = baseFootPositionWalk[leg_idx] + (bodyVelocity * (walkParams.stepTime * 0.5f));

  // Calculate the angle the body will rotate during half a stance phase
  float rotationAngle = bodyAngularVelocityYaw * walkParams.stepTime * 0.5f;
  float cos_ra = cosf(rotationAngle);
  float sin_ra = sinf(rotationAngle);

  // Store original linear target X before calculating rotated target X
  float original_linear_x = linearTargetTouchdownPos.x;
  // Store original linear target Y (needed for correct Y calculation)
  float original_linear_y = linearTargetTouchdownPos.y;

  // Calculate the final rotated target touchdown position
  Vec3 targetTouchdownPos; // Declare the final target variable
  targetTouchdownPos.x = original_linear_x * cos_ra - original_linear_y * sin_ra;
  targetTouchdownPos.y = original_linear_x * sin_ra + original_linear_y * cos_ra;

  // Set the Z coordinate from the commanded base position (not rotated)
  targetTouchdownPos.z = baseFootPositionWalk[leg_idx].z;

  // --- Interpolate Position ---
  // Z Position (Vertical Lift): Use the bell curve for smooth up-and-down motion.
  float lift_curve = bell_curve_lift(gaitProgress);
  
  leg.currentPosition.z = mapf(gaitProgress, 0.0f, 1.0f, leg.swingStartPosition.z, baseFootPositionWalk[leg_idx].z) 
                      + walkParams.stepHeight * lift_curve;

  // XY Position: Use quintic interpolation for smooth horizontal motion.
  // We interpolate from the recorded lift-off point (leg.swingStartPosition)
  // to the calculated target landing point (targetTouchdownPos).
  // Assume zero velocity *relative to the Ground, not Walk Frame* at the start and end of the swing to avoid slippage
  leg.currentPosition.x = quintic_interpolate_pos(
      leg.swingStartPosition.x, targetTouchdownPos.x,
      -bodyVelocity.x, -bodyVelocity.x, // Zero start/end velocity relative to the ground X
      walkParams.stepTime, time_in_swing);

  leg.currentPosition.y = quintic_interpolate_pos(
      leg.swingStartPosition.y, targetTouchdownPos.y,
      -bodyVelocity.y, -bodyVelocity.y, // Zero start/end velocity relative to the ground Y
      walkParams.stepTime, time_in_swing);
}

void updateWalkCycle(float dt) {
    if (!walkCycleRunning) {
        Serial.println("!walkCycleRunning");
        // If walking is stopped, do nothing. Legs hold their last commanded position.
        return;
    }

    // 1. Compute the new leg positions based on the walkState
    switch (walkState){
      case IdleStance: {
        for(uint8_t i = 0; i < 6; ++i) leg_stance(dt, i);

        if(bodyVelocity.x || bodyVelocity.y || bodyVelocity.z || bodyAngularVelocityYaw) {
          walkState = TripodSwing1;
          gaitProgress = 0.0f;
        }
      } break;

      case TripodSwing1: {
        gaitProgress += dt / walkParams.stepTime;
        if(gaitProgress >= 1.0f) gaitProgress = 1.0f;

        leg_swing(dt, 0);
        leg_swing(dt, 2);
        leg_swing(dt, 4);

        leg_stance(dt, 1);
        leg_stance(dt, 3);
        leg_stance(dt, 5);

        if(gaitProgress >= 1.0f){
          gaitProgress = 0.0f;
          walkState = TripodSwing2;

          if(!(bodyVelocity.x || bodyVelocity.y || bodyVelocity.z || bodyAngularVelocityYaw)) {
            bool noStuffToMove = true;
            for(uint8_t i = 0; i < 6 && noStuffToMove; ++i){
              if(legCycleData[i].currentPosition != baseFootPositionWalk[i]) noStuffToMove = false;
            }
            if(noStuffToMove) walkState = IdleStance;
          }
        }
      } break;

      case TripodSwing2: {
        gaitProgress += dt / walkParams.stepTime;
        if(gaitProgress >= 1.0f) gaitProgress = 1.0f;

        leg_swing(dt, 1);
        leg_swing(dt, 3);
        leg_swing(dt, 5);

        leg_stance(dt, 0);
        leg_stance(dt, 2);
        leg_stance(dt, 4);

        if(gaitProgress >= 1.0f){
          gaitProgress = 0.0f;
          walkState = TripodSwing1;

          if(!(bodyVelocity.x || bodyVelocity.y || bodyVelocity.z || bodyAngularVelocityYaw)) {
            bool noStuffToMove = true;
            for(uint8_t i = 0; i < 6 && noStuffToMove; ++i){
              if(legCycleData[i].currentPosition != baseFootPositionWalk[i]) noStuffToMove = false;
            }
            if(noStuffToMove) walkState = IdleStance;
          }
        }
      } break;

      default: {
        Serial.println("no_walk_mode");
      } break;
    }

    // 2. Transform coordinates and apply the new positions
    for (uint8_t leg_idx = 0; leg_idx < LEG_COUNT; leg_idx++) {
        LegCycleData& leg = legCycleData[leg_idx]; // Get reference to this leg's state

        // 2.1. Transform Walk Frame Target to Leg IK Frame Target
        Vec3 P_foot_leg_ik_input; // This will hold the coordinates for calculateIK
        transformWalkFrameToLegFrame(leg.currentPosition, leg_idx, P_foot_leg_ik_input);

        // 2.2. Perform Inverse Kinematics
        float coxa_rad, femur_rad, tibia_rad;
        bool ik_success = calculateIK(leg_idx,
                                      P_foot_leg_ik_input.x,
                                      P_foot_leg_ik_input.y,
                                      P_foot_leg_ik_input.z,
                                      coxa_rad, femur_rad, tibia_rad); // Outputs

        // 2.3. Send Commands to Servos if IK Succeeded
        if (ik_success) {
            // Retrieve the correct servo channel numbers for this leg/joint
            uint8_t coxa_servo_channel = LEG_SERVOS[leg_idx][0];
            uint8_t femur_servo_channel = LEG_SERVOS[leg_idx][1];
            uint8_t tibia_servo_channel = LEG_SERVOS[leg_idx][2];

            // Send the calculated angles (in radians) to the low-level servo control function
            setAngleRadians(coxa_servo_channel, coxa_rad);
            setAngleRadians(femur_servo_channel, femur_rad);
            setAngleRadians(tibia_servo_channel, tibia_rad);
        }

    } // End loop through legs
}