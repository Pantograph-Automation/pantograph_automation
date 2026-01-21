#include <AccelStepper.h>

// ----- Pins -----
#define J1_PULSE_PIN 9
#define J1_DIR_PIN 8
#define J1_ENABLE_PIN 10  // Optional; HIGH = enabled

#define J2_PULSE_PIN 11
#define J2_DIR_PIN 12
#define J2_ENABLE_PIN 13  // Optional; HIGH = enabled

// ----- Stepper configuration -----
AccelStepper J1(AccelStepper::DRIVER, J1_PULSE_PIN, J1_DIR_PIN);
AccelStepper J2(AccelStepper::DRIVER, J2_PULSE_PIN, J2_DIR_PIN);
// ----- Motion parameters -----
const long stepsPerRev = 1600;       // your driver setting
const long forwardRevs = 2;          // number of revolutions forward
const long backwardRevs = 0;         // number of revolutions backward

void setup() {
    Serial.begin(9600);

    // Enable driver
    pinMode(J1_ENABLE_PIN, OUTPUT);
    digitalWrite(J1_ENABLE_PIN, HIGH);  // Enable driver

    pinMode(J2_ENABLE_PIN, OUTPUT);
    digitalWrite(J2_ENABLE_PIN, HIGH);  // Enable driver


    // Motion settings
    J1.setMaxSpeed(800);       // ~0.5 rev/sec at 1600 PPR
    J1.setAcceleration(600);   // smooth acceleration
    J2.setMaxSpeed(800);
    J2.setAcceleration(600);

    J1.moveTo(100);
    J2.moveTo(100);
}

void loop() {
    // Check if motors are still moving

    // if (J1.distanceToGo() != 0 || J2.distanceToGo() != 0) {
    //     J1.run();
    //     J2.run();
    // } else {
    //     // Reverse direction for next move
    //     if (J1.currentPosition() == 100 && J2.currentPosition() == 100) {
    //         // Move back to start
    //         J1.moveTo(0);
    //         J2.moveTo(0);
    //         Serial.println("Reversing back to 0");
    //     } else if (J1.currentPosition() == 0 && J2.currentPosition() == 0) {
    //         // Move 45° forward again
    //         J1.moveTo(100);
    //         J2.moveTo(100);
    //         Serial.println("Moving 45 degrees forward");
    //     }
    // }
}

