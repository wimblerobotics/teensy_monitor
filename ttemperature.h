#pragma once

#include <stdint.h>

#include "tmodule.h"

class TTemperature : TModule {
public:

  static const uint8_t NUMBER_SENSORS = 2;

  static TTemperature& singleton();

  int16_t getValueTenthsC(uint8_t index);

  void loop();

  const char* name() { return "TTemperature"; }

  void setup();

private:

  typedef enum {
    ANALOG_0_PIN = 24,
    ANALOG_1_PIN = 25
  } TAnalogPins;

  TTemperature();

  static int16_t g_analog0Value;
  static int16_t g_analog1Value;
  
  static TTemperature* g_singleton;
};