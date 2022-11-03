#pragma once

#include <Adafruit_INA260.h>
#include <stdint.h>

#include "tmodule.h"

class TMotorCurrent : TModule {
 public:
  // Which motor.
  typedef enum Motor { kLeft, kRight, kNumberMotors } Motor;

  // Get the average motor current value.
  int GetValueMa(Motor index);

  // Singleton constructor.
  static TMotorCurrent& singleton();

 protected:
  // From TModule.‰
  void loop();

  // From TModule.
  const char* name() { return "Amps"; }

  // From TModule.
  void setup();

 private:
  // GPIO pins for current sensors.
  typedef enum {
    kLeftMotorAddress = 0x40,
    kRightMotorAddress = 0x41
  } MOTOR_PINS;

  // Private constructor.
  TMotorCurrent();

  // Singleton instance.
  static TMotorCurrent* g_singleton_;

  // Number of samples to go into an average motor current.
  static const uint8_t kAverageCount = 20;

  // Motor current sensor instances.
  static Adafruit_INA260 g_left_INA260_;
  static Adafruit_INA260 g_right_INA260_;

  // Indicate if motor current sensor was detected.
  static bool g_have_left_motor_sensor_;
  static bool g_have_right_motor_sensor_;

  // Reported value for motor current sensors.
  static float g_left_motor_current_ma_;
  static float g_right_motor_current_ma_;

  // Index for insertion of next motor current sample value.
  static uint8_t g_next_motor_current_index_;

  // Rolling window of motor current sensor values by sensor.
  static float g_left_motor_current_ma_readings_[kAverageCount];
  static float g_right_motor_current_ma_readings_[kAverageCount];
};