#ifndef WALK_CYCLE_H
#define WALK_CYCLE_H

#include <math.h>       // For fmod, M_PI, etc.
#include "ik.h"         // For calculateIK
#include "utils.h"      // For Vec3, clampf, setAngleRadians, etc.
#include "robot_spec.h" // For LEG_COUNT, defaultFootPositionWalk, bodyVelocity, etc.
#include "body_transform.h" // For transformWalkFrameToLegFrame

// --- Walk Cycle Parameters ---
// These parameters control the gait characteristics.
// They can be modified dynamically (e.g., by serial or remote commands).
struct WalkParams {
  float stepHeight = 3.0f;
  float stepFrequency = 1.0f;
  float dutyFactor = 0.5f;
};

// --- Per-Leg Walk Cycle State ---
// Stores state information specific to each leg's movement within the cycle.
struct LegCycleData {
  //the latest foot position calculated
  Vec3 currentPosition;

  // Position of the foot in the Walk Frame {X,Y,Z} (cm) at the exact
  // moment it lifted off the ground to begin the swing phase.
  // Used as the starting point for swing trajectory interpolation.
  Vec3 swingStartPosition;
};

extern WalkParams walkParams;
extern LegCycleData legCycleData[LEG_COUNT];
extern bool walkCycleRunning;
extern float globalPhase;

// --- Function Declarations ---
void setupWalkcycle();
void updateWalkCycle(float dt); // Main update function
float bell_curve_lift(float t); // Helper for smooth vertical motion
float quintic_interpolate_pos(float p0, float p1, float v0, float v1, float T, float t); // Helper for smooth XY motion

// --- Global Variable Definitions (Example - could be in a .cpp file) ---
// It's generally better practice to define these in a .cpp file (like walkcycle.cpp or robot_spec.cpp)
// and declare them 'extern' here. But for simplicity in a single-header context:
WalkParams walkParams; // Use default values from struct definition initially
LegCycleData legCycleData[LEG_COUNT];
bool walkCycleRunning = false;
float globalPhase = 0.0f;
// bodyVelocity is defined in robot_spec.cpp

// --- Function Implementations ---

/**
 * @brief Initializes the walk cycle state for each leg.
 * Does NOT move the legs; just sets up internal data structures.
 */
void setupWalkcycle() {
  walkCycleRunning = false; // Ensure walk cycle is stopped initially
  globalPhase = 0.0f;       // Reset phase

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


/**
 * @brief Main update function for the hexapod walk cycle.
 * Calculates target foot positions in the Walk Frame based on phase and velocity,
 * transforms them into the Leg IK Frame considering body pose,
 * performs IK, and sends commands to servos.
 *
 * @param dt Time elapsed since the last update (in seconds).
 */
void updateWalkCycle(float dt) {
    if (!walkCycleRunning) {
        // If walking is stopped, do nothing. Legs hold their last commanded position.
        return;
    }

    // 1. Update Global Phase
    globalPhase = fmodf(globalPhase + walkParams.stepFrequency * dt, 1.0f);
    if (globalPhase < 0.0f) globalPhase += 1.0f; // Ensure phase is [0, 1)

    // 2. Calculate Cycle Timing Parameters
    // Avoid division by zero if frequency is extremely low
    float T_cycle = (walkParams.stepFrequency > 1e-6f) ? 1.0f / walkParams.stepFrequency : 1e6f;
    // Clamp duty factor to prevent issues, ensure stance/swing times are positive
    float duty_factor = clampf(walkParams.dutyFactor, 0.01f, 0.99f);
    float T_stance = duty_factor * T_cycle;
    float T_swing = T_cycle - T_stance;

    // 3. Loop Through Each Leg
    for (uint8_t i = 0; i < LEG_COUNT; i++) {
        LegCycleData& leg = legCycleData[i]; // Get reference to this leg's state
        //Vec3 P_foot_walk;                    // Calculated target position in Walk Frame for this update

        // Determine leg-specific phase (can add offsets here later for different gaits)
        // Simple example: Tripod gait - Legs 0, 2, 4 step together, Legs 1, 3, 5 step together
        bool isTripodGroup1 = (i == 0 || i == 2 || i == 4); // Example Grouping (BR, FR, CL)
        float phaseOffset = isTripodGroup1 ? 0.0f : 0.5f; // Group 2 is half a cycle offset
        float legPhase = fmodf(globalPhase + phaseOffset, 1.0f);
        if (legPhase < 0.0f) legPhase += 1.0f;

        // 4. Calculate Target Foot Position in Walk Frame

        if (legPhase < duty_factor) {
            // --- STANCE PHASE ---
            // Foot is on the ground and should move backward relative to the body/Walk frame
            // to remain stationary relative to the Ground.

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

        } else {
            // --- SWING PHASE ---
            // Foot is in the air, moving from swingStartPosition to the next target touchdown position.

            // The normalized progress through the swing phase (0.0 to 1.0)
            float swingPhase = (legPhase - duty_factor) / (1.0f - duty_factor);
            
            // The time elapsed *within* the current swing phase
            float time_in_swing = swingPhase * T_swing;

            // The target touchdown position for the *end* of this swing phase.
            // This is where the *next* stance phase will begin. (Same calculation as stanceTouchdownPos above)
            Vec3 linearTargetTouchdownPos = baseFootPositionWalk[i] + (bodyVelocity * (T_stance * 0.5f));

            // Calculate the angle the body will rotate during half a stance phase
            float rotationAngle = bodyAngularVelocityYaw * T_stance * 0.5f;
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
            targetTouchdownPos.z = baseFootPositionWalk[i].z;

            // --- Interpolate Position ---
            // Z Position (Vertical Lift): Use the bell curve for smooth up-and-down motion.
            float lift_curve = bell_curve_lift(swingPhase);
            
            leg.currentPosition.z = mapf(swingPhase, 0.0f, 1.0f, leg.swingStartPosition.z, baseFootPositionWalk[i].z) 
                                + walkParams.stepHeight * lift_curve;

            // XY Position: Use quintic interpolation for smooth horizontal motion.
            // We interpolate from the recorded lift-off point (leg.swingStartPosition)
            // to the calculated target landing point (targetTouchdownPos).
            // Assume zero velocity *relative to the Ground, not Walk Frame* at the start and end of the swing to avoid slippage
            leg.currentPosition.x = quintic_interpolate_pos(
                leg.swingStartPosition.x, targetTouchdownPos.x,
                -bodyVelocity.x, -bodyVelocity.x, // Zero start/end velocity relative to the ground X
                T_swing, time_in_swing);

            leg.currentPosition.y = quintic_interpolate_pos(
                leg.swingStartPosition.y, targetTouchdownPos.y,
                -bodyVelocity.y, -bodyVelocity.y, // Zero start/end velocity relative to the ground Y
                T_swing, time_in_swing);
        }

        // 5. Transform Walk Frame Target to Leg IK Frame Target
        Vec3 P_foot_leg_ik_input; // This will hold the coordinates for calculateIK
        transformWalkFrameToLegFrame(leg.currentPosition, i, P_foot_leg_ik_input);

        // 6. Perform Inverse Kinematics
        float coxa_rad, femur_rad, tibia_rad;
        bool ik_success = calculateIK(i, // Leg index
                                      P_foot_leg_ik_input.x,
                                      P_foot_leg_ik_input.y,
                                      P_foot_leg_ik_input.z,
                                      coxa_rad, femur_rad, tibia_rad); // Outputs

        // 7. Send Commands to Servos if IK Succeeded
        if (ik_success) {
            // Retrieve the correct servo channel numbers for this leg/joint
            uint8_t coxa_servo_channel = LEG_SERVOS[i][0];
            uint8_t femur_servo_channel = LEG_SERVOS[i][1];
            uint8_t tibia_servo_channel = LEG_SERVOS[i][2];

            // Send the calculated angles (in radians) to the low-level servo control function
            setAngleRadians(coxa_servo_channel, coxa_rad);
            setAngleRadians(femur_servo_channel, femur_rad);
            setAngleRadians(tibia_servo_channel, tibia_rad);
        }

    } // End loop through legs
}

#endif // WALK_CYCLE_H