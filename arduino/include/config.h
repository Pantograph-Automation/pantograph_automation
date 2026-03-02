
// Joint 1 configuration
#define J1_PULSE_PIN 14
#define J1_DIR_PIN 11
#define J1_ENABLE_PIN 10
#define J1_LIMIT_PIN 9
#define J1_POS_AT_CALIB 0.0f

// Joint 2 configuration
#define J2_PULSE_PIN 22
#define J2_DIR_PIN 21
#define J2_ENABLE_PIN 20
#define J2_LIMIT_PIN 2
#define J2_POS_AT_CALIB 0.0f

// Universal configuration constant
#define UPDATE_RATE 1000
#define RAD_PER_STEP 0.003926991f
#define PULSE_WIDTH_US 10UL
#define V_MAX 10.0f                   // rad/s max
#define ACCEL_MAX 0.1f              // rad/s^2
#define POS_DEADBAND 0.002f           // radians (~0.1 deg)

#define RADIANS_TO_DEGREES 57.29578
#define DEGREES_TO_RADIANS 0.01745329

#define Kp 10.0f
#define Ki 0.0f
#define Kd 0.0f
