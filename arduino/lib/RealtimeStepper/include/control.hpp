#pragma once
#include "Arduino.h"

template <typename T>
class Buffer {
  private:
    T* buffer;     // Pointer to the array
    int capacity;  // Maximum size
    int head = 0;
    int tail = 0;
    int count = 0;

  public:
    Buffer(int size) : capacity(size) {
      buffer = new T[capacity]; 
    }

    ~Buffer() {
      delete[] buffer;
    }

    bool push(const T& value) {
      if (count == capacity) return false;
      buffer[head] = value;
      head = (head + 1) % capacity;
      count++;
      return true;
    }

    T pop() {
      if (count == 0) return T();
      T value = buffer[tail];
      tail = (tail + 1) % capacity;
      count--;
      return value;
    }

    T get(int index) const {
        if (index >= count) {
            return T(); // Out of bounds, return default
        }
        
        // Calculate the actual position in the internal array
        int actualIndex = (tail + index) % capacity;
        return buffer[actualIndex];
    }

    void pushOverwrite(const T& value) {
      if (count == capacity) {
        tail = (tail + 1) % capacity;
      } else {
        count++;
      }
      
      data[head] = value;
      head = (head + 1) % capacity;
    }

    int size() const { return count; }
  };

class Stepper
{
  public:

    Stepper() {};

    // Rotational state information
    //
    inline float get_position(int index = 0) { get_(position_buffer, index); }

    inline float get_velocity(int index = 0) { get_(velocity_buffer, index); }

    inline float get_acceleration(int index = 0) { get_(acceleration_buffer, index); }

    inline void update_position(float position) { set_(position_buffer, position); }

    inline void update_velocity(float velocity) { set_(velocity_buffer, velocity); }

    inline void update_acceleration(float acceleration) { set_(acceleration_buffer, acceleration); }

    // Time information
    //
    inline unsigned long get_time(int index = 0) { get_(time_buffer, index); }

    inline unsigned long get_time_delta()
    {
      return get_time() - get_time(1); // t - (t-1)
    }

    inline void update_time()
    {
      time_buffer.pushOverwrite(micros());
    }

    // Non-blocking pulse method
    //


  private:

    // Buffers that remember the states at t, t-1, and t-2
    Buffer<float> position_buffer{3};
    Buffer<float> velocity_buffer{3};
    Buffer<float> acceleration_buffer{3};
    Buffer<unsigned long> time_buffer{3}; // t, t-1, and t-2

    bool pulse_active;


    // Generic getter and setter
    template<typename T>
    inline T get_(Buffer<T>& buffer, int index)
    {
      return buffer.get(index);
    }

    template<typename T>
    inline void set_(Buffer<T>& buffer, T& value)
    {
      buffer.pushOverwrite(value);
    }




};

class Controller
{
  public:

    enum class Status
    {
      SUCCESS,
      ERROR_UNINITIALIZED,
      ERROR_COLLISION,
      ERROR
    };

    void init(const int number_of_steppers)
    {
      dof_ = number_of_steppers;
      
      steppers_ = new Stepper[dof_];
      
    }

    inline Status pid(
      
    ) {

      if(!initialized()) return Status::ERROR_UNINITIALIZED;
      
      // vel_cmd = Kp (e(k) - e(k - 1)) + Ki e(k) dt + Kd * (e(k) - 2e(k - 1) + e(k-2)) / (dt)
      float vel_cmd;
      float Kp;
      float Ki;
      float Kd;
      bool pulse_active;
      unsigned long pulse_active;

      for (int i = 0; i < dof_; i++)
      {
        steppers_[i].update_time();

        float dt = steppers_[i].get_time_delta() * 1e-6f;

        if (pulse_active && (now - pulse_start >= PULSE_WIDTH_US)) {
          digitalWrite(S1_PULSE_PIN, LOW);
          pulse_active = false;
        }

      }

      // ---- Pulse width housekeeping (non-blocking) ----
      

      // ---- read encoder for PID (measured position) ----
      float current_angle = read_encoder(offset);
      // ---- PID
      error = desired_position - current_angle;

      // deadband around target
      if (fabs(error) < POS_DEADBAND) {
        error = 0.0f;
      }

      float vel_cmd = Kp * error;

      if (vel_cmd - vel_prev > ACCEL_MAX) vel_cmd = vel_prev + ACCEL_MAX;
      if (vel_cmd - vel_prev < -ACCEL_MAX) vel_cmd = vel_prev - ACCEL_MAX;
      
      vel_prev = vel_cmd;

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
        } else if (delta <= -RAD_PER_STEP) {
          // backward step
        }
      }
            

    }


  private:

    int dof_ = 0;

    Stepper* steppers_;    

    inline bool initialized()
    {
      if (
        dof_ == 0 ||
        nullptr == steppers_
      ) return false; else return true;
    }


    float* commanded_velocity;
    float* commanded_
    


};