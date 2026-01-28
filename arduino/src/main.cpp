#include "main.h"

// usbipd list
// usbipd attach --wsl --busid 2-1

// Encoder E1(E1_A_PIN, E2_B_PIN);
int encoder_reading = 0;

void setup() {
    Serial.begin(9600);

    pinMode(E1_A_PIN, INPUT_PULLUP);

    delay(500);
}

void loop() {

    // encoder_reading = E1.read();
    // Serial.print("Encoder reading: ");
    // Serial.println(encoder_reading);
    // delay(50);

    // Serial.flush();

    encoder_reading = digitalRead(E1_A_PIN);
    Serial.print("Pin reading: ");
    Serial.println(encoder_reading);
    delay(300);
    Serial.flush();
}

