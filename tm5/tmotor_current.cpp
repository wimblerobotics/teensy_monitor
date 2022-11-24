#include "tmotor_current.h"

#include <Adafruit_INA260.h>
#include <stdint.h>

#include "tconfiguration.h"
#include "tmicro_ros.h"

TMotorCurrent::TMotorCurrent() : TModule(TModule::kMotorCurrent) {
  for (int8_t i = 0; i < kAverageCount; i++) {
    g_left_motor_current_ma_readings_[i] = 0.0;
    g_right_motor_current_ma_readings_[i] = 0;
  }
}

int TMotorCurrent::GetValueMa(Motor index) {
  if (index == kLeft)
    return g_left_motor_current_ma_;
  else
    return g_right_motor_current_ma_;
}

void TMotorCurrent::loop() {
  if (g_have_left_motor_sensor_) {
    g_left_motor_current_ma_readings_[g_next_motor_current_index_] =
        g_left_INA260_.readCurrent();
    if (TM5::kDoDetailDebug) {
      char msg[128];
      snprintf(msg, sizeof(msg),
               "INFO [TMotorCurrent::loop] left current: %-2.3f",
               g_left_motor_current_ma_readings_[g_next_motor_current_index_]);
      TMicroRos::singleton().PublishDiagnostic(msg);
    }
  } else {
    if (TM5::kDoDetailDebug) {
      TMicroRos::singleton().PublishDiagnostic(
          "INFO [TMotorCurrent::loop] no left current sensor");
    }
    g_left_motor_current_ma_readings_[g_next_motor_current_index_] = 0;
  }

  if (g_have_right_motor_sensor_) {
    g_right_motor_current_ma_readings_[g_next_motor_current_index_] =
        g_right_INA260_.readCurrent();
    if (TM5::kDoDetailDebug) {
      char msg[128];
      snprintf(msg, sizeof(msg),
               "INFO [TMotorCurrent::loop] right current: %-2.3f",
               g_left_motor_current_ma_readings_[g_next_motor_current_index_]);
      TMicroRos::singleton().PublishDiagnostic(msg);
    }
  } else {
    if (TM5::kDoDetailDebug) {
      TMicroRos::singleton().PublishDiagnostic(
          "INFO [TMotorCurrent::loop] no right current sensor");
    }
    g_right_motor_current_ma_readings_[g_next_motor_current_index_] = 0;
  }

  g_next_motor_current_index_++;
  if (g_next_motor_current_index_ >= kAverageCount) {
    g_next_motor_current_index_ = 0;
  }

  float leftAveragaMa = 0.0;
  float rightAveragaMa = 0.0;
  for (uint8_t i = 0; i < kAverageCount; i++) {
    leftAveragaMa += g_left_motor_current_ma_readings_[i];
    rightAveragaMa += g_right_motor_current_ma_readings_[i];
  }

  g_left_motor_current_ma_ = leftAveragaMa / kAverageCount;
  g_right_motor_current_ma_ = rightAveragaMa / kAverageCount;
}

void TMotorCurrent::setup() {
  if (TM5::kDoDetailDebug) {
    TMicroRos::singleton().PublishDiagnostic("INFO [TMotorCurrent::setup]");
  }

  g_have_left_motor_sensor_ = g_left_INA260_.begin(kLeftMotorAddress);
  g_have_right_motor_sensor_ = g_right_INA260_.begin(kRightMotorAddress);
  if (!g_have_left_motor_sensor_) {
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR TMotorCurrent::setup no left motor current sensor");
  }

  if (!g_have_right_motor_sensor_) {
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR TMotorCurrent::setup no right motor current sensor");
  }
}

TMotorCurrent& TMotorCurrent::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TMotorCurrent();
  }

  return *g_singleton_;
}

TMotorCurrent* TMotorCurrent::g_singleton_ = nullptr;

uint8_t TMotorCurrent::g_next_motor_current_index_ = 0;

bool TMotorCurrent::g_have_left_motor_sensor_;
bool TMotorCurrent::g_have_right_motor_sensor_;

Adafruit_INA260 TMotorCurrent::g_left_INA260_;
Adafruit_INA260 TMotorCurrent::g_right_INA260_;

float TMotorCurrent::g_left_motor_current_ma_ = 0;

float TMotorCurrent::g_right_motor_current_ma_ = 0;

float TMotorCurrent::g_left_motor_current_ma_readings_[kAverageCount];

float TMotorCurrent::g_right_motor_current_ma_readings_[kAverageCount];