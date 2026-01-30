#include <AccelStepper.h>
#include <Encoder.h>
#include <AS5600.h>

#define S1_PULSE_PIN 9
#define S1_DIR_PIN 8
#define S1_ENABLE_PIN 10
#define RAD_PER_STEP 0.003926991f
#define PULSE_WIDTH_US 10UL
#define V_MAX 500.0f                   // rad/s max commanded velocity
#define POS_DEADBAND 0.005f           // radians (~0.1 deg)

#define RADIANS_TO_DEGREES 57.29578
#define DEGREES_TO_RADIANS 0.01745329
