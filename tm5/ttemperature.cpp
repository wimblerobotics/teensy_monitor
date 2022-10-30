#include "ttemperature.h"

#include <stdint.h>

#include "Arduino.h"
#include "tmicro_ros.h"

int16_t TTemperature::getValueTenthsC(TEMPERATURE device) {
  switch (device) {
    case LEFT:
      return g_left_motor_temperature_tenthsC_;

    case RIGHT:
      return g_right_motor_temperature_tenthsC_;

    default:
      return -1;
  }
}

void TTemperature::loop() {
  static uint32_t start_time = millis();

  int raw = analogRead(ANALOG_0_PIN);
  float tempMv = (raw * 3250 / 1024.0) - 55.0;  // TMP36 temperature conversion.
  g_left_motor_temperature_tenthsC_ = (tempMv - 500);
  g_averages_[0][g_next_average_index_] = g_left_motor_temperature_tenthsC_;

  raw = analogRead(ANALOG_1_PIN);
  tempMv = (raw * 3250 / 1024.0) - 55.0;
  g_right_motor_temperature_tenthsC_ = (tempMv - 500);
  g_averages_[1][g_next_average_index_] = g_right_motor_temperature_tenthsC_;

  g_next_average_index_ += 1;
  if (g_next_average_index_ >= kNumberReadingsToAverage) {
    g_next_average_index_ = 0;
  }

  uint32_t now = millis();
  if ((now - start_time) > 500) {
    float left_motor_average_sum = 0;
    float right_motor_average_sum = 0;
    for (size_t reading = 0; reading < kNumberReadingsToAverage; reading++) {
      left_motor_average_sum += g_averages_[0][reading];
      right_motor_average_sum += g_averages_[1][reading];
    }

    TMicroRos::singleton().publishTemperature(
        "left_motor",
        (left_motor_average_sum / kNumberReadingsToAverage) / 10.0);
    TMicroRos::singleton().publishTemperature(
        "right_motor",
        (right_motor_average_sum / kNumberReadingsToAverage) / 10.0);
    start_time = now;
  }
}

void TTemperature::setup() { analogReadResolution(10); }

TTemperature::TTemperature() : TModule(TModule::kTEMPERATURE) {
  for (size_t device = 0; device < 2; device++) {
    for (size_t reading = 0; reading < kNumberReadingsToAverage; reading++) {
      g_averages_[device][reading] = 0.0;
    }
  }

  g_next_average_index_ = 0;
}

TTemperature& TTemperature::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TTemperature();
  }

  return *g_singleton_;
}

int16_t TTemperature::g_left_motor_temperature_tenthsC_ = 0;
int16_t TTemperature::g_right_motor_temperature_tenthsC_ = 0;

float TTemperature::g_averages_[2][kNumberReadingsToAverage];
size_t TTemperature::g_next_average_index_ = 0;

TTemperature* TTemperature::g_singleton_ = nullptr;