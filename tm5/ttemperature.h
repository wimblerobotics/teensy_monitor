#pragma once

#include <stdint.h>

#include "tmodule.h"

class TTemperature : TModule {
 public:
  // Which temperature sensor.
  typedef enum Temperature {
    kLeft,
    kRight,
    kNumberTemperatures  // Number of temperature sensors.
  } Temperature;

  int16_t GetValueTenthsC(Temperature device);

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
  enum { kAnalog0Pin = 26, kAnalog1Pin = 27 };

  // Private constructor.
  TTemperature();

  // Last temperature sensor readings.
  static int16_t g_left_motor_temperature_tenthsC_;
  static int16_t g_right_motor_temperature_tenthsC_;

  static const uint8_t kNumberReadingsToAverage_ = 50;
  static float g_averages_[2][kNumberReadingsToAverage_];
  static size_t g_next_average_index_;

  // Singleton instance.
  static TTemperature* g_singleton_;
};