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