#pragma once

#include "Arduino.h"
#include "ring_buffer.hpp"
#include "AS5600.h"
#define POS_CMD_MAX_ERROR 0.019634955f
#define WIRE_CLOCK 400000

struct Pair {

  Pair() {};
  Pair(float value, unsigned long time) {
    this->value = value;
    this->time = time;
  };
  float value;
  unsigned long time;
};

class Joint
{
  public:

    /**
     * @param pulse_pin_ Pin used to pulse the stepper motor
     * @param enable_pin_ Pin used to enable the stepper motor
     * @param direction_pin_ Pin used to control the direction of the motor
     */
    Joint(int pulse_pin, int enable_pin, int direction_pin, int limit_pin, TwoWire *wire = &Wire)
    {
      pulse_pin_ = pulse_pin;
      enable_pin_ = enable_pin;
      direction_pin_ = direction_pin;

      limit_pin_ = limit_pin;

      wire_ = wire;
      encoder_ = AS5600(wire);

    };

    /**
     * @brief Initialize pins, buffers, and I2C interface
     */
    void begin() {

      pinMode(pulse_pin_, OUTPUT);
      pinMode(enable_pin_, OUTPUT);
      pinMode(direction_pin_, OUTPUT);

      position_buffer_.fill(Pair(0.0, 0UL));

      wire_->begin();
      wire_->setClock(WIRE_CLOCK);
    }

    /**
     * @brief Update state information (realtime, reads micros and encoder)
     */
    inline void update_state()
    {
      if(!calibrated_) {
        Serial.println("Not calibrated!! Cannot update joint state.");
        delay(100);
        return;
      }

      float new_measured_position = read_encoder();
      
      if (new_measured_position - last_measured_position_ < -PI) total_rotations_++;
      else if(new_measured_position - last_measured_position_ > PI) total_rotations_--;

      last_absolute_position_.value = (total_rotations_ * 2*PI) + new_measured_position;
      last_absolute_position_.time = micros();

      position_buffer_.pushOverwrite(last_absolute_position_);
    }

    /**
     * @brief Compute the PID law, updating commanded position (used when deciding whether or not to step)
     * @param
     */
    inline void update_control(
      const float desired_position,
      const float Kp,
      const float max_velocity,
      const float max_acceleration,
      const float deadband)
    {
      float error = desired_position - position_buffer_.get(0).value;

      // deadband around target
      if (fabs(error) < deadband) {
        error = 0.0f;
      }

      // compute and clamp commanded velocity
      float vel_cmd = Kp * error;
      clamp_(vel_cmd, -max_velocity, max_velocity);

      // integrate velocity to get commanded position
      position_cmd_ += vel_cmd * position_buffer_.get(0).time - position_buffer_.get(1).time;

      if (position_cmd_ - position_est_ > POS_CMD_MAX_ERROR)
          position_cmd_ = position_est_ + POS_CMD_MAX_ERROR;
      else if (position_cmd_ - position_est_ < -POS_CMD_MAX_ERROR)
          position_cmd_ = position_est_ - POS_CMD_MAX_ERROR;
    }

    /**
     * @brief Checks if a step is required based on the estimated and commanded position
     * @param position_cmd The commanded stepper position
     * @param position_est The estimated stepper position
     * @details Place this method at the beginning of the control loop
     */
    inline void check_step_required(const float rad_per_step)
    {
      float delta = position_cmd_ - position_est_;

      if (!pulse_active_) {
        if (delta >= rad_per_step) {
          Serial.println("Pulse required!");
          set_direction_forward();
          digitalWrite(pulse_pin_, HIGH);
          pulse_start_ = micros();
          pulse_active_ = true;
          position_est_ += rad_per_step;
        } else if (delta <= -rad_per_step) {
          set_direction_backward();
          digitalWrite(pulse_pin_, HIGH);
          pulse_start_ = micros();
          pulse_active_ = true;
          position_est_ -= rad_per_step;
        }
      }
    };

    /**
     * @brief Pulses if a step is required AND enough time has passed to pulse
     * @param pulse_width The time between writing HIGH and LOW for a pulse
     * @details Place this method at the beginning of the control loop
     */
    inline void pulse_if_required(const unsigned long pulse_width)
    {
      if (pulse_active_ && (micros() - pulse_start_ >= pulse_width)) {
        digitalWrite(pulse_pin_, LOW);
        pulse_active_ = false;
      }
    }

    /**
     * Utility
     */
    float read_encoder() {
      return encoder_.rawAngle() * AS5600_RAW_TO_RADIANS;
    }
    void enable() {
      digitalWrite(enable_pin_, HIGH);
    }
    void disable() {
      digitalWrite(enable_pin_, LOW);
    }
    void set_direction_forward() {
      digitalWrite(direction_pin_, HIGH);
    }
    void set_direction_backward() {
      digitalWrite(direction_pin_, LOW);
    }

  private:

    // pin information
    int pulse_pin_;
    int enable_pin_;
    int direction_pin_;
    int limit_pin_;

    // encoder
    TwoWire* wire_;
    AS5600 encoder_;
    float offset_;
    bool calibrated_ = false;

    // position buffer
    Buffer<Pair> position_buffer_{3};
    Pair last_absolute_position_;

    // phase wrapping implementation
    float last_measured_position_;
    int total_rotations_;

    // estimated and commanded positions
    float position_est_;
    float position_cmd_;

    // Pulse housekeeping
    bool pulse_active_;
    unsigned long pulse_start_;
    
    template <typename T>
    inline T clamp_(T val, T low, T high)
    {
      if (val <= low) return low;
      if (val >= high) return high;
      else return val;
    }


};