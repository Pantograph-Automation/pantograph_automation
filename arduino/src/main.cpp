#include <AS5600.h>
#include "control.hpp"
#include "config.h"

constexpr unsigned long DT = 1000000UL / UPDATE_RATE; // Microseconds
// usbipd list
// usbipd attach --wsl --busid 2-1

unsigned long last_cycle_time;

Joint J1(J1_PULSE_PIN, J1_ENABLE_PIN, J1_DIR_PIN, J1_LIMIT_PIN, &Wire);

Controller controller(J1);

void setup()
{
  Serial.begin(11520);

  while(!Serial);

  J1.begin();


  // stepper.disable();
  // Serial.println("Calibrating...");
  // delay(2000);
  // offset = calibrate(100, 20);
  // Serial.println("Calibrated!! Move the arm.");
  // delay(2000);
  // stepper.enable();

  Serial.println("Joint 1 initialized");
  Serial.println("Move to calibrated position");
  J1.disable();
  delay(2000);
  J1.enable();
  Serial.println("Callibrating J1...");
  J1.calibrate(J1_POS_AT_CALIB);
    
}


void loop()
{

  J1.pulse_if_required(PULSE_WIDTH_US);
  J1.update_state();
  
  while(micros() - last_cycle_time <= DT);

  
  // Run the PID law to determine whether a step is required
  Status status = controller.pid(
    Kp, Ki, Kd,
    0.5,
    DT,
    V_MAX,
    ACCEL_MAX,
    POS_CMD_MAX_ERROR,
    POS_DEADBAND,
    RAD_PER_STEP
  );

  last_cycle_time = micros();
  
  if (status == Status::FINISHED)
  {
    Serial.println("Finished moving to setpoint!");
  }
}