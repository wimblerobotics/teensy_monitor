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
  static int16_t g_left_motor_temperature_tenthsC;
  static int16_t g_right_motor_temperature_tenthsC;

  // Singleton instance.
  static TTemperature* g_singleton;
};