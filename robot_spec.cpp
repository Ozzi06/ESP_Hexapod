#include "robot_spec.h"

#ifdef SCRAP_MECHANIC
const float COXA_LENGTH  = 8.0f; // Length from hip to knee joint
const float FEMUR_LENGTH = 9.0; // Length from knee to ankle joint
const float TIBIA_LENGTH = 10.5; // Length from ankle to foot
#else
// Leg mechanical parameters (in cm)
const float COXA_LENGTH  = 7.0f; // Length from hip to knee joint
const float FEMUR_LENGTH = 9.4; // Length from knee to ankle joint
const float TIBIA_LENGTH = 12.0; // Length from ankle to foot
#endif
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


float latestServoAngles[LEG_COUNT * 3] = {0};

// Default neutral foot positions in Walk Frame {X, Y, Z} (in cm)
// Relative to the point on the ground directly below the body center. (walk frame)
#ifdef OSSIAN_HEMMA
Vec3 baseFootPositionWalk[LEG_COUNT] = {
    { 27.0f, -19.0f, 0.0f}, // Leg 0 (BR)
    { 32.0f,   0.0f, 0.0f}, // Leg 1 (CR)
    { 27.0f,  19.0f, 0.0f}, // Leg 2 (FR)
    {-27.0f, -19.0f, 0.0f}, // Leg 3 (BL)
    {-32.0f,   0.0f, 0.0f}, // Leg 4 (CL)
    {-27.0f,  19.0f, 0.0f}  // Leg 5 (FL)
};
#else
Vec3 baseFootPositionWalk[LEG_COUNT] = {
    { 20.0f, -22.0f, 0.0f}, // Leg 0 (BR)
    { 27.0f,   0.0f, 0.0f}, // Leg 1 (CR)
    { 20.0f,  22.0f, 0.0f}, // Leg 2 (FR)
    {-20.0f, -22.0f, 0.0f}, // Leg 3 (BL)
    {-27.0f,   0.0f, 0.0f}, // Leg 4 (CL)
    {-20.0f,  22.0f, 0.0f}  // Leg 5 (FL)
};
#endif

#ifdef OSSIAN_HEMMA
const Vec3 legOriginOffset[LEG_COUNT] = {
    {   12.0f,   -10.0f, 0.0f}, // Leg 0(BR)
    {   12.0f,     0.0f, 0.0f}, // Leg 1(CR)
    {   12.0f,    10.0f, 0.0f}, // Leg 2(FR)
    {  -12.0f,   -10.0f, 0.0f}, // Leg 3(BL)
    {  -12.0f,     0.0f, 0.0f}, // Leg 4(CL)
    {  -12.0f,    10.0f, 0.0f}  // Leg 5(FL)
};
#else
const Vec3 legOriginOffset[LEG_COUNT] = {
    {   5.75f,   -7.85f , 0.0f}, // Leg 0(BR)
    {   7.9f,      0.0f, 0.0f}, // Leg 1(CR)
    {   5.75f,    7.85f, 0.0f}, // Leg 2(FR)
    {  -5.75f,   -7.85f, 0.0f}, // Leg 3(BL)
    {  -7.9f,      0.0f, 0.0f}, // Leg 4(CL)
    {  -5.75f,    7.85f, 0.0f}  // Leg 5(FL)
};
#endif

const float legMountingAngle[LEG_COUNT] = {
    -1*M_PI / 4.0f, // Leg 0(BR)
     0*M_PI / 4.0f, // Leg 1(CR)
    +1*M_PI / 4.0f, // Leg 2(FR)
    
    -3*M_PI / 4.0f, // Leg 3(BL)
    +4*M_PI / 4.0f, // Leg 4(CL)
    +3*M_PI / 4.0f  // Leg 5(FL)
};

const float servo_center_angle[3] = {
    0.0f, 0.0f, -45.0f * M_PI / 180.0f, // Coxa, Femur, Tibia
};

// Define global state variables (initial values)
Vec3 bodyPositionOffset = {0.0f, 0.0f, 0.0f}; // Start 0cm above walk frame origin to avoid smashing the ground on startup
Quaternion bodyOrientation = Quaternion::identity();
Vec3 bodyVelocity = {0.0f, 0.0f, 0.0f}; // Initialize to zero velocity
float bodyAngularVelocityYaw = 0.0f;

