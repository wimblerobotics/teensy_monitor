#include "ttemperature.h"

#include <stdint.h>

#include "Arduino.h"
#include "tmicro_ros.h"

int16_t TTemperature::getValueTenthsC(TEMPERATURE device) {
  switch (device) {
    case LEFT:
      return g_left_motor_temperature_tenthsC;

    case RIGHT:
      return g_right_motor_temperature_tenthsC;

    default:
      return -1;
  }
}

void TTemperature::loop() {
  static uint32_t start_time = millis();

  int raw = analogRead(ANALOG_0_PIN);
  float tempMv = (raw * 3250 / 1024.0) - 55.0;  // TMP36 temperature conversion.
  g_left_motor_temperature_tenthsC = (tempMv - 500);

  raw = analogRead(ANALOG_1_PIN);
  tempMv = (raw * 3250 / 1024.0) - 55.0;
  g_right_motor_temperature_tenthsC = (tempMv - 500);

  uint32_t now = millis();
  if ((now - start_time) > 500) {
    TMicroRos::singleton().publishTemperature(
        "left_motor", g_left_motor_temperature_tenthsC / 10.0);
    TMicroRos::singleton().publishTemperature(
        "right_motor", g_right_motor_temperature_tenthsC / 10.0);
    start_time = now;
  }
}

void TTemperature::setup() { analogReadResolution(10); }

TTemperature::TTemperature() : TModule(TModule::kTEMPERATURE) {}

TTemperature& TTemperature::singleton() {
  if (!g_singleton) {
    g_singleton = new TTemperature();
  }

  return *g_singleton;
}

int16_t TTemperature::g_left_motor_temperature_tenthsC = 0;
int16_t TTemperature::g_right_motor_temperature_tenthsC = 0;

TTemperature* TTemperature::g_singleton = nullptr;