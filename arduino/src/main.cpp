#include "main.h"

// usbipd list
// usbipd attach --wsl --busid 2-1

const float POS_CMD_MAX_ERROR = 5 * RAD_PER_STEP;
float error = 0.0f;
float prev_error = 0.0f;

// Encoder E1(E1_A_PIN, E2_B_PIN);
AS5600 as5600;

uint32_t clk = 0;

uint32_t start, stop;
float offset;
bool pulse_active = false;
unsigned long pulse_start = 0UL;
unsigned long last_time = 0UL;
// PID loop variables
float Kp = 20.0;
float Ki = 0.0;
float Kd = 0.0;
float pos_cmd = 0.0;
float pos_est = 0.0f;

const float desired_position = 1.570796; // 90 degrees

void enable_stepper() {
  digitalWrite(S1_ENABLE_PIN, HIGH);
}

void disable_stepper() {
  digitalWrite(S1_ENABLE_PIN, LOW);
}
void set_direction_forward()
{
  digitalWrite(S1_DIR_PIN, HIGH);
}

void set_direction_backward()
{
  digitalWrite(S1_DIR_PIN, LOW);
}

float calibrate(const int sample_size, const unsigned long ms_delay)
{
  float offset_ = 0.0;

  for (int i = 0; i < sample_size; i++)
  {
    offset_ +=  as5600.rawAngle() * AS5600_RAW_TO_RADIANS;
    delay(ms_delay);
  }
  offset_ = offset_ / (float)sample_size;
  return offset_;
}

float read_encoder(float offset_)
{

  float radians = as5600.rawAngle() * AS5600_RAW_TO_RADIANS - offset_;

  if (radians < 0)
  {
    radians += 2*PI;
  }

  return radians;
}

void setup()
{
  Serial.begin(11520);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(S1_PULSE_PIN, OUTPUT);
  pinMode(S1_ENABLE_PIN, OUTPUT);
  pinMode(S1_DIR_PIN, OUTPUT);

  as5600.begin(4);  //  set direction pin.

  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  disable_stepper();
  Serial.println("Calibrating...");
  delay(2000);
  offset = calibrate(100, 20);
  Serial.println("Calibrated!! Move the arm.");
  delay(2000);
  enable_stepper();
  
}


void loop()
{
  
  unsigned long now = micros();
  unsigned long elapsed_us = now - last_time;
  float dt = elapsed_us * 1e-6f; // seconds
  last_time = now;

  // ---- Pulse width housekeeping (non-blocking) ----
  if (pulse_active && (now - pulse_start >= PULSE_WIDTH_US)) {
    digitalWrite(S1_PULSE_PIN, LOW);
    pulse_active = false;
  }

  // // ---- I2C clock (your code) ----
  // clk += 100000;
  // if (clk > 800000) clk = 100000;
  // Wire.setClock(clk);

  // ---- read encoder for PID (measured position) ----
  float current_angle = read_encoder(offset);
  // ---- PID
  error = desired_position - current_angle;

  // deadband around target
  if (fabs(error) < POS_DEADBAND) {
    error = 0.0f;
  }

  float vel_cmd = Kp * error;       // rad/s from PID (add I/D as needed)
  // clamp velocity to something feasible
  if (vel_cmd > V_MAX) vel_cmd = V_MAX;
  if (vel_cmd < -V_MAX) vel_cmd = -V_MAX;

  // ---- integrate velocity to get a commanded continuous position ----
  pos_cmd += vel_cmd * dt;

  float cmd_err = pos_cmd - pos_est;
  if (cmd_err > POS_CMD_MAX_ERROR)
      pos_cmd = pos_est + POS_CMD_MAX_ERROR;
  else if (cmd_err < -POS_CMD_MAX_ERROR)
      pos_cmd = pos_est - POS_CMD_MAX_ERROR;

  // ---- decide whether to step, using internal pos_est (not raw encoder) ----
  float delta = pos_cmd - pos_est;

  if (!pulse_active) {
    if (delta >= RAD_PER_STEP) {
      // forward step
      set_direction_forward();  // set correct direction polarity for your hardware
      digitalWrite(S1_PULSE_PIN, HIGH);
      pulse_start = now;
      pulse_active = true;
      pos_est += RAD_PER_STEP;         // update internal estimate immediately
    } else if (delta <= -RAD_PER_STEP) {
      // backward step
      set_direction_backward();
      digitalWrite(S1_PULSE_PIN, HIGH);
      pulse_start = now;
      pulse_active = true;
      pos_est -= RAD_PER_STEP;
    }
  }
}