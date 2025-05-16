#include "math_utils.h"
#include <math.h>   // For fabsf (used in mapf's division check)
#include <stdio.h>  // For fprintf (if mapf's warning were enabled)


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  float in_span = in_max - in_min;
  if (fabsf(in_span) < 1e-9f) { // Check for division by zero or near-zero
    // fprintf(stderr, "Warning: mapf division by zero/near-zero (in_min approx in_max).\n");
    return out_min; // Or some other sensible default, e.g., (out_min + out_max) / 2.0f
  }
  return (x - in_min) * (out_max - out_min) / in_span + out_min;
}

float clampf(float val, float min_val, float max_val){
  if (val < min_val) val = min_val;
  if (val > max_val) val = max_val;
  return val;
}

float clampmapf(float x, float in_min, float in_max, float out_min, float out_max){
  float val = mapf(x, in_min, in_max, out_min, out_max);
  // The original sequential clamping logic:
  if (val > out_max) val = out_max;
  if (val < out_min) val = out_min;
  return val;
}

Quaternion slerp(const Quaternion& qa, const Quaternion& qb, float t) {
    // Ensure t is clamped between 0 and 1
    t = clampf(t, 0.0f, 1.0f);

    // Calculate dot product (cosine of angle between quaternions)
    float cos_half_theta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;

    Quaternion q_result;
    Quaternion qb_temp = qb; // Temporary qb to handle antipodal case

    // If qb is on the opposite hemisphere, use its antipodal representation
    // to ensure the shortest path is taken.
    if (cos_half_theta < 0.0f) {
        qb_temp.w = -qb.w; qb_temp.x = -qb.x; qb_temp.y = -qb.y; qb_temp.z = -qb.z;
        cos_half_theta = -cos_half_theta;
    }

    // If quaternions are very close, perform linear interpolation (LERP)
    // and normalize to avoid division by zero from sin(angle).
    if (cos_half_theta > 0.9999f) { // Arbitrary threshold for "very close"
        q_result.w = qa.w + t * (qb_temp.w - qa.w);
        q_result.x = qa.x + t * (qb_temp.x - qa.x);
        q_result.y = qa.y + t * (qb_temp.y - qa.y);
        q_result.z = qa.z + t * (qb_temp.z - qa.z);
        q_result.normalize();
    } else {
        // Standard SLERP
        float half_theta = acosf(cos_half_theta);
        float sin_half_theta = sinf(half_theta);

        // if sin_half_theta is 0, means an angle of 0 or PI.
        // The >0.9999f check above should handle angle=0.
        // If angle is PI, result is undefined without more context, but LERP is often fine.
        if (fabsf(sin_half_theta) < 0.0001f) { // Should be rare due to previous check
             q_result.w = qa.w + t * (qb_temp.w - qa.w);
             q_result.x = qa.x + t * (qb_temp.x - qa.x);
             q_result.y = qa.y + t * (qb_temp.y - qa.y);
             q_result.z = qa.z + t * (qb_temp.z - qa.z);
        } else {
            float ratio_a = sinf((1.0f - t) * half_theta) / sin_half_theta;
            float ratio_b = sinf(t * half_theta) / sin_half_theta;

            q_result.w = ratio_a * qa.w + ratio_b * qb_temp.w;
            q_result.x = ratio_a * qa.x + ratio_b * qb_temp.x;
            q_result.y = ratio_a * qa.y + ratio_b * qb_temp.y;
            q_result.z = ratio_a * qa.z + ratio_b * qb_temp.z;
        }
        // SLERP should produce a normalized quaternion if inputs are normalized.
        // For safety, you could normalize here, but it's often not strictly needed
        // if ratio calculations are precise.
        // q_result.normalize(); 
    }
    return q_result;
}