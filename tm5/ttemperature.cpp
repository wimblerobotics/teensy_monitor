#include "ttemperature.h"

#include <stdint.h>

#include "Arduino.h"
#include "tmicro_ros.h"

int16_t TTemperature::GetValueTenthsC(Temperature device) {
  switch (device) {
    case kLeft:
      return g_left_motor_temperature_tenthsC_;

    case kRight:
      return g_right_motor_temperature_tenthsC_;

    default:
      return -1;
  }
}

void TTemperature::loop() {
  static uint32_t start_time_ms = millis();

  int raw = analogRead(kAnalog0Pin);
  float temp_mv = (raw * 3250 / 1024.0) - 55.0;  // TMP36 temperature conversion.
  g_left_motor_temperature_tenthsC_ = (temp_mv - 500);
  g_averages_[0][g_next_average_index_] = g_left_motor_temperature_tenthsC_;

  raw = analogRead(kAnalog1Pin);
  float battery_v = raw * 0.08845 * 0.91;
  g_averages_[1][g_next_average_index_] = battery_v;

  g_next_average_index_ += 1;
  if (g_next_average_index_ >= kNumberReadingsToAverage_) {
    g_next_average_index_ = 0;
  }

  uint32_t now_ms = millis();
  if ((now_ms - start_time_ms) > 500) {
    float left_motor_average_sum = 0;
    float voltage_sum = 0;
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      left_motor_average_sum += g_averages_[0][reading];
      voltage_sum += g_averages_[1][reading];
    }

    TMicroRos::singleton().PublishTemperature(
        "left_motor",
        (left_motor_average_sum / kNumberReadingsToAverage_) / 10.0);
    TMicroRos::singleton().PublishBattery(
        "main_battery", voltage_sum / kNumberReadingsToAverage_);
    start_time_ms = now_ms;
  }
}

void TTemperature::setup() { analogReadResolution(10); }

TTemperature::TTemperature() : TModule(TModule::kTemperature) {
  for (size_t device = 0; device < 2; device++) {
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
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

float TTemperature::g_averages_[2][kNumberReadingsToAverage_];
size_t TTemperature::g_next_average_index_ = 0;

TTemperature* TTemperature::g_singleton_ = nullptr;