#include <AS5600.h>
#include "joint.hpp"
#include "config.h"

constexpr unsigned long DT = 1000000UL / UPDATE_RATE; // Microseconds
// usbipd list
// usbipd attach --wsl --busid 2-1

unsigned long last_cycle_time;

Joint J1(J1_PULSE_PIN, J1_ENABLE_PIN, J1_DIR_PIN, J1_LIMIT_PIN, &Wire);

void setup()
{

  Serial.begin(11520);

  while(!Serial);

  J1.begin();

  Serial.println("Joint 1 initialized");
  Serial.println("Move to calibrated position");
  J1.disable();
  delay(2000);
  J1.enable();
  Serial.println("Calibrated...");

    
}


void loop()
{
  
}