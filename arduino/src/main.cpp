#include <AccelStepper.h>
#include <Encoder.h>

#include "main.h"

// ----- Stepper configuration -----
AccelStepper S1(AccelStepper::DRIVER, S1_PULSE_PIN, S1_DIR_PIN);
AccelStepper S2(AccelStepper::DRIVER, S2_PULSE_PIN, S2_DIR_PIN);


// ----- Motion parameters -----
const long stepsPerRev = 1600;       // your driver setting
const long forwardRevs = 2;          // number of revolutions forward
const long backwardRevs = 0;         // number of revolutions backward

void setup() {
    Serial.begin(115200);

    // Enable driver
    pinMode(S1_ENABLE_PIN, OUTPUT);
    digitalWrite(S1_ENABLE_PIN, HIGH);  // Enable driver

    pinMode(S2_ENABLE_PIN, OUTPUT);
    digitalWrite(S2_ENABLE_PIN, HIGH);  // Enable driver


    // Motion settings
    S1.setMaxSpeed(800);       // ~0.5 rev/sec at 1600 PPR
    S1.setAcceleration(600);   // smooth acceleration
    S2.setMaxSpeed(800);
    S2.setAcceleration(600);

    S1.moveTo(100);
    S2.moveTo(100);
}

void loop() {
    // Check if motors are still moving

    // if (S1.distanceToGo() != 0 || S2.distanceToGo() != 0) {
    //     S1.run();
    //     S2.run();
    // } else {
    //     // Reverse direction for next move
    //     if (S1.currentPosition() == 100 && S2.currentPosition() == 100) {
    //         // Move back to start
    //         S1.moveTo(0);
    //         S2.moveTo(0);
    //         Serial.println("Reversing back to 0");
    //     } else if (S1.currentPosition() == 0 && S2.currentPosition() == 0) {
    //         // Move 45° forward again
    //         S1.moveTo(100);
    //         S2.moveTo(100);
    //         Serial.println("Moving 45 degrees forward");
    //     }
    // }
}

