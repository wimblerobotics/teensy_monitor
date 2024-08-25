#include "ttemperature.h"

#include <stdint.h>

#include "Arduino.h"
#include "tmicro_ros.h"

void TTemperature::loop() {
  static uint32_t start_time_ms = millis();

  float raw = analogRead(kAnalog1Pin);
  float battery_v = raw * 0.08845 * 0.91;
  g_averages_[1][g_next_average_index_] = battery_v;

  g_next_average_index_ += 1;
  if (g_next_average_index_ >= kNumberReadingsToAverage_) {
    g_next_average_index_ = 0;
  }

  uint32_t now_ms = millis();
  if ((now_ms - start_time_ms) > 500) {
    float voltage_sum = 0;
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      voltage_sum += g_averages_[1][reading];
    }

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

float TTemperature::g_averages_[2][kNumberReadingsToAverage_];
size_t TTemperature::g_next_average_index_ = 0;

TTemperature* TTemperature::g_singleton_ = nullptr;