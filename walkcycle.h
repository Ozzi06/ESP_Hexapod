#ifndef WALK_CYCLE_H
#define WALK_CYCLE_H

#include "ik.h"
#include "utils.h"

// Walk cycle parameters
struct WalkParams {
  float stepHeight = 2.0f;     // Maximum height of step (cm)
  float stepFrequency = 0.75f;  // Steps per second
  float dutyFactor = 0.5f;     // Fraction of cycle spent on ground (0-1)
};

// Per-leg walk cycle data
struct LegCycleData {
  Vec3 basePosition;          // Where the leg should be relative to body
  Vec3 currentPosition;       // Where the leg actually is
  Vec3 swingStartPosition;
  bool active = false;
};

// Global variables
WalkParams walkParams;
LegCycleData legCycleData[LEG_COUNT];
bool walkCycleRunning = false;
float globalPhase = 0.0f;      // Global phase for coordination
Vec3 bodyVelocity;            // Desired body velocity vector (cm/s)

void setupWalkcycle() {
  setupIK();
  walkCycleRunning = false;
  
  // Initialize leg cycle data
  for (uint8_t i = 0; i < LEG_COUNT; i++) {
    legCycleData[i].basePosition = {25, 0, -8};
    legCycleData[i].currentPosition = legCycleData[i].basePosition;
    legCycleData[i].active = true;
  }
}

float bell_curve_lift(float t) {
    t = clampf(t, 0.0f, 1.0f);
    float t_minus_1 = 1.0f - t;
    return 64.0f * (t * t * t) * (t_minus_1 * t_minus_1 * t_minus_1);
}

float quintic_interpolate_pos(float p0, float p1, float v0, float v1, float T, float t) {
    //t = clampf(t, 0.0f, T);
    if (T <= 1e-9f) return p0; // Avoid division by zero
    float tn = t / T, tn2 = tn*tn, tn3 = tn2*tn, tn4 = tn3*tn, tn5 = tn4*tn;
    float h00 = 1.0f - 10.0f*tn3 + 15.0f*tn4 - 6.0f*tn5;
    float h10 = tn - 6.0f*tn3 + 8.0f*tn4 - 3.0f*tn5;
    float h01 = 10.0f*tn3 - 15.0f*tn4 + 6.0f*tn5;
    float h11 = -4.0f*tn3 + 7.0f*tn4 - 3.0f*tn5;
    return h00*p0 + h10*T*v0 + h01*p1 + h11*T*v1;
}

void updateWalkCycle(float dt) {
  
  globalPhase = fmod(globalPhase + walkParams.stepFrequency * dt, 1.0f);
  // Calculate Cycle Timing
    float T_cycle = 1.0f / walkParams.stepFrequency;
    float T_stance = walkParams.dutyFactor * T_cycle;
    float T_swing = T_cycle - T_stance;
  
  for (uint8_t i = 0; i < LEG_COUNT; i++) {
    if (!legCycleData[i].active) continue;
    
    LegCycleData& leg = legCycleData[i];
    
    
    // Calculate leg-specific phase (offset for alternating legs)
    float legPhase = fmod(globalPhase + 0.0f, 1.0f);
    
    if(legPhase > walkParams.dutyFactor){
      float stepPhase = (legPhase - walkParams.dutyFactor) / walkParams.dutyFactor;
      
      //float smooth_stepupdown = (1-cos(2*M_PI*stepPhase))/2;
      float smooth_stepupdown = bell_curve_lift(stepPhase);
      leg.currentPosition.z = leg.basePosition.z + walkParams.stepHeight * pow(smooth_stepupdown, 1.0f);

      
      // XY Position: Quintic interpolation
      float predictionTime = T_stance * 0.5f;
      Vec3 targetTouchdownPos = leg.basePosition; // Start from neutral XY
      targetTouchdownPos.x -= bodyVelocity.x * predictionTime;
      targetTouchdownPos.y -= bodyVelocity.y * predictionTime;


      Vec3 swingVelXY = bodyVelocity * -1.0f; // Velocity relative to body

      float timeInSwing = stepPhase * T_swing;



      leg.currentPosition.x = quintic_interpolate_pos(
          leg.swingStartPosition.x, targetTouchdownPos.x,
          bodyVelocity.x * 1.0f, bodyVelocity.x * 1.5f, T_swing, timeInSwing); // 1.5 to make it a bit more agressive, not really correct but it works
      
      leg.currentPosition.y = quintic_interpolate_pos(
          leg.swingStartPosition.y, targetTouchdownPos.y,
          bodyVelocity.y * 1.0f, bodyVelocity.y * 1.5f, T_swing, timeInSwing);

      //leg.currentPosition.x += ((leg.basePosition.x - bodyVelocity.x / 2) - leg.currentPosition.x) * decayFactor * decayrate;
      //leg.currentPosition.y += ((leg.basePosition.y - bodyVelocity.y / 2) - leg.currentPosition.y) * decayFactor * decayrate;
      
    }
    else {
      // Update current position according to body velocity
      leg.currentPosition.x += bodyVelocity.x * dt;
      leg.currentPosition.y += bodyVelocity.y * dt;

      leg.swingStartPosition = leg.currentPosition;
    }

    // Update IK target
    legTargets[i] = leg.currentPosition;
    moveLegToTarget(i, false);
    
  }
}


#endif