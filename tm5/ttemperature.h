#pragma once

#include <stdint.h>

#include "tmodule.h"

class TTemperature : TModule {
 public:
  // Which temperature sensor.
  typedef enum TEMPERATURE {
    LEFT,
    RIGHT,
    NUMBER_TEMPERATURES  // Number of temperature sensors.
  } TEMPERATURE;

  int16_t getValueTenthsC(TEMPERATURE device);

  // Singleton constructor.
  static TTemperature& singleton();

 protected:
  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "Temp"; }

  // From TModule.
  void setup();

 private:
  // GPIO addresses of temperature sensors.
  enum { ANALOG_0_PIN = 26, ANALOG_1_PIN = 27 };

  // Private constructor.
  TTemperature();

  // Last temperature sensor readings.
  static int16_t g_left_motor_temperature_tenthsC_;
  static int16_t g_right_motor_temperature_tenthsC_;

  static const uint8_t kNumberReadingsToAverage = 50;
  static float g_averages_[2][kNumberReadingsToAverage];
  static size_t g_next_average_index_;

  // Singleton instance.
  static TTemperature* g_singleton_;
};