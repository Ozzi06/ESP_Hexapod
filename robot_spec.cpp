#include "robot_spec.h"

// Leg mechanical parameters (in cm)
const float COXA_LENGTH  = 7.0f; // Length from hip to knee joint
const float FEMUR_LENGTH = 10.0; // Length from knee to ankle joint
const float TIBIA_LENGTH = 11.6; // Length from ankle to foot

// Joint angle limits (in radians)
const float COXA_MIN_ANGLE  =  (-60 * M_PI / 180.0);
const float COXA_MAX_ANGLE  =   (60 * M_PI / 180.0);
const float FEMUR_MIN_ANGLE =  (-70 * M_PI / 180.0);
const float FEMUR_MAX_ANGLE =   (70 * M_PI / 180.0);
const float TIBIA_MIN_ANGLE =  (-70 * M_PI / 180.0);
const float TIBIA_MAX_ANGLE =   (70 * M_PI / 180.0);

// ... other lengths and angle limits ...

//Leg names
const char* leg_names[LEG_COUNT] = {"Back Right", "Mid Right", "Front Right", "Back Left", "Mid Left", "Front Left"};


// Default neutral foot positions in Walk Frame {X, Y, Z} (in cm)
// Relative to the point on the ground directly below the body center. (walk frame)
const Vec3 defaultFootPositionWalk[LEG_COUNT] = {
    { 15.0f, -13.0f, 0.0f}, // Leg 0 (BR): Example - Back Right
    { 20.0f,   0.0f, 0.0f}, // Leg 1 (CR): Example - Center Right
    { 15.0f,  13.0f, 0.0f}, // Leg 2 (FR): Example - Front Right
    {-15.0f, -13.0f, 0.0f}, // Leg 3 (BL): Example - Back Left
    {-20.0f,   0.0f, 0.0f}, // Leg 4 (CL): Example - Center Left
    {-15.0f,  13.0f, 0.0f}  // Leg 5 (FL): Example - Front Left
};

// Servo channel assignments per leg [leg][joint: 0=coxa, 1=femur, 2=tibia]
const uint8_t LEG_SERVOS[LEG_COUNT][3] = {
  { 9, 10, 11},   // Leg 0 (BR)
  {12, 13, 14},   // Leg 1 (CR)
  {15, 16, 17},   // Leg 2 (FR)
  { 0,  1,  2},   // Leg 3 (BL)
  { 3,  4,  5},   // Leg 4 (CL)
  { 6,  7,  8},   // Leg 5 (FL)
};

const Vec3 legOriginOffset[LEG_COUNT] = {
    {  0.0f,   0.0f, 0.0f}, // Leg 0(BR)
    {  0.0f,   0.0f, 0.0f}, // Leg 1(CR)
    {  0.0f,   0.0f, 0.0f}, // Leg 2(FR)
    {  0.0f,   0.0f, 0.0f}, // Leg 3(BL)
    {  0.0f,   0.0f, 0.0f}, // Leg 4(CL)
    {  0.0f,   0.0f, 0.0f}  // Leg 5(FL)
};

const float legMountingAngle[LEG_COUNT] = {
    -1*M_PI / 4.0f, // Leg 0(BR)
     0*M_PI / 4.0f, // Leg 1(CR)
    +1*M_PI / 4.0f, // Leg 2(FR)
    
    -3*M_PI / 4.0f, // Leg 3(BL)
    +2*M_PI / 4.0f, // Leg 4(CL)
    +3*M_PI / 4.0f  // Leg 5(FL)
};

// Define global state variables (initial values)
Vec3 bodyPositionOffset = {0.0f, 0.0f, 10.0f}; // Start 15cm above walk frame origin
Quaternion bodyOrientation = Quaternion::identity();
Vec3 bodyVelocity = {0.0f, 0.0f, 0.0f}; // Initialize to zero velocity

