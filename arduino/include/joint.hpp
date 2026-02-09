#pragma once

#include "Arduino.h"
#include "ring_buffer.hpp"
#include "AS5600.h"

#define WIRE_CLOCK 400000
#define DEFAULT_BUFFER_FILL 0.0f

#define VEL_UPDATE_WIDTH 0.002f
#define ACC_UPDATE_WIDTH 0.01f

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

      Pair default_buffer_fill(DEFAULT_BUFFER_FILL,  micros());

      position_buffer_.fill(default_buffer_fill);
      velocity_buffer_.fill(default_buffer_fill);
      acceleration_buffer_.fill(default_buffer_fill);

      wire_->begin();
      wire_->setClock(WIRE_CLOCK);
    }

    /**
     * @brief Calibrate the encoder at a certain position
     */
    void calibrate(const float position_at_calibrate, const int sample_size=20, const unsigned long ms_delay=5)
    {
      offset_ = 0.0;
      for (int i = 0; i < sample_size; i++)
      {
        offset_ +=  encoder_.rawAngle() * AS5600_RAW_TO_RADIANS;
        delay(ms_delay);
      }
      offset_ = offset_ / (float)sample_size - position_at_calibrate;

      calibrated_ = true;      
    }

    // Basic stepper utility methods
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

    // Basic encoder utility methods
    float read_position() {
      float radians = encoder_.rawAngle() * AS5600_RAW_TO_RADIANS - offset_;

      if (radians <= -PI)
      {
        radians += 2*PI;
      }
      return radians;
    }

    // Buffer updating methods
    inline Pair get_position(int index = 0) { return get_(position_buffer_, index); }

    inline Pair get_velocity(int index = 0) { return get_(velocity_buffer_, index); }

    inline Pair get_acceleration(int index = 0) { return get_(acceleration_buffer_, index); }

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

      unsigned long current_time = micros();

      // Read position once and timestamp it with current_time
      last_position.value = read_position();
      last_position.time  = current_time;
      update_(position_buffer_, last_position);

      unsigned long dt_pos_us = (current_time - pos_at_last_vel_reading_.time);
      if (dt_pos_us == 0) dt_pos_us = 1;

      float delta_pos = last_position.value - pos_at_last_vel_reading_.value;
      float velocity = delta_pos / ((float)dt_pos_us * 1e-6f);

      // If movement is below noise/threshold, treat as zero velocity and update buffer
      if (fabsf(delta_pos) < VEL_UPDATE_WIDTH) {
        // only update buffer if we need to change reported velocity to zero
        if (fabsf(get_velocity().value) > 0.0f) {
          last_velocity.value = 0.0f;
          last_velocity.time  = current_time;
          update_(velocity_buffer_, last_velocity);
        }
        // reset baseline so future dt is measured from the static position
        pos_at_last_vel_reading_ = last_position;
      } else {
        // significant movement -> publish computed velocity and update baseline
        last_velocity.value = velocity;
        last_velocity.time  = current_time;
        update_(velocity_buffer_, last_velocity);

        pos_at_last_vel_reading_ = last_position; // use this pos as new baseline
      }

      // ---------- ACCELERATION ----------
      // Compute acceleration relative to the baseline velocity used for last acceleration update
      unsigned long dt_vel_us = (current_time - vel_at_last_acc_reading_.time);
      if (dt_vel_us == 0) dt_vel_us = 1;

      float delta_vel = last_velocity.value - vel_at_last_acc_reading_.value;
      float acceleration = delta_vel / ((float)dt_vel_us * 1e-6f); // units/sec^2

      if (fabsf(delta_vel) < ACC_UPDATE_WIDTH) {
        // if small change, report zero acceleration (if it isn't already zero)
        if (fabsf(get_acceleration().value) > 0.0f) {
          last_acceleration.value = 0.0f;
          last_acceleration.time  = current_time;
          update_(acceleration_buffer_, last_acceleration);
        }
        // reset baseline velocity so future dt is measured from this (possibly zero) velocity
        vel_at_last_acc_reading_ = last_velocity;
      } else {
        // significant change -> publish computed acceleration and update baseline
        last_acceleration.value = acceleration;
        last_acceleration.time  = current_time;
        update_(acceleration_buffer_, last_acceleration);

        vel_at_last_acc_reading_ = last_velocity;
      }
    }


    /**
     * @brief Checks if a step is required based on the estimated and commanded position
     * @param position_cmd The commanded stepper position
     * @param position_est The estimated stepper position
     * @details Place this method at the beginning of the control loop
     */
    inline void check_step_required(const float position_cmd, const float rad_per_step, float position_est)
    {
      float delta = position_cmd - position_est;

      if (!pulse_active_) {
        if (delta >= rad_per_step) {
          Serial.println("Pulse required!");
          set_direction_forward();
          digitalWrite(pulse_pin_, HIGH);
          pulse_start_ = micros();
          pulse_active_ = true;
          position_est += rad_per_step;
        } else if (delta <= -rad_per_step) {
          set_direction_backward();
          digitalWrite(pulse_pin_, HIGH);
          pulse_start_ = micros();
          pulse_active_ = true;
          position_est -= rad_per_step;
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

    // State buffers
    Buffer<Pair> position_buffer_{3};
    Pair last_position;

    Buffer<Pair> velocity_buffer_{3};
    Pair last_velocity;
    Pair pos_at_last_vel_reading_;

    Buffer<Pair> acceleration_buffer_{3};
    Pair last_acceleration;
    Pair vel_at_last_acc_reading_;
    

    // Pulse housekeeping
    bool pulse_active_;
    unsigned long pulse_start_;

    /**
     * @brief Generic getter
     */
    template<typename T>
    inline T get_(Buffer<T>& buffer, int index)
    {
      return buffer.get(index);
    }

    /**
     * @brief Generic setter
     */
    template<typename T>
    inline void update_(Buffer<T>& buffer, const T& value)
    {
      buffer.pushOverwrite(value);
    }

};