#include "tsonar.h"

#include <stdint.h>

#include "Arduino.h"
#include "TimerOne.h"
#include "Wire.h"
#include "tmicro_ros.h"

void TSonar::CommonInterruptHandler(uint8_t pin, long& end_time,
                                    long& start_time, uint8_t& average_index,
                                    size_t sonar_index) {
  switch (digitalRead(pin)) {
    case HIGH:
      end_time = 0;
      start_time = micros();
      break;

    case LOW:
      end_time = micros();
      g_values_mm_[sonar_index] =
          (end_time - start_time) * g_time_to_mm_scaler_;
      g_values_mm_history_[sonar_index][average_index++] =
          g_values_mm_[sonar_index];
      if (average_index >= kNumberReadingsToAverage) {
        average_index = 0;
      }

      int averageSum = 0;
      for (size_t i = 0; i < kNumberReadingsToAverage; i++) {
        averageSum += g_values_mm_history_[sonar_index][i];
      }

      g_average_value_m_[sonar_index] =
          (averageSum * 0.001) / kNumberReadingsToAverage;

      TMicroRos::PublishSonar(sonar_index, g_average_value_m_[sonar_index]);
      break;
  }
}

float TSonar::GetAverageValueM(Sonar device) {
  return g_average_value_m_[device];
}

void TSonar::Echo0InterruptHandler() {
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho0, end_time, start_time, average_index, 0);
}

void TSonar::Echo1InterruptHandler() {
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho1, end_time, start_time, average_index, 1);
}

void TSonar::Echo2InterruptHandler() {
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho2, end_time, start_time, average_index, 2);
}

void TSonar::Echo3InterruptHandler() {
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho3, end_time, start_time, average_index, 3);
}

int TSonar::GetValueMm(Sonar device) {
  if (static_cast<int>(device) >= kNumberSonars) {
    return -1;
  } else {
    return g_values_mm_[static_cast<int>(device)];
  }
}

void TSonar::loop() {
  // const int kAlertDistanceMm = 3 * 25.4;

  // for (uint8_t i = 0; i < kNumberSonars; i++) {
  //   if (doStopMotorsOnCollisionThreat &&
  //       (GetValueMm(static_cast<Sonar>(i)) < kAlertDistanceMm)) {
  //     // TAlert::singleton().set(map[i]);
  //   } else {
  //     // TAlert::singleton().reset(map[i]);
  //   }
  // }
}

void TSonar::setup() {
  pinMode(kPinEcho0, INPUT);
  pinMode(kPinTrigger0, OUTPUT);
  pinMode(kPinEcho1, INPUT);
  pinMode(kPinTrigger1, OUTPUT);
  pinMode(kPinEcho2, INPUT);
  pinMode(kPinTrigger2, OUTPUT);
  pinMode(kPinEcho3, INPUT);
  pinMode(kPinTrigger3, OUTPUT);
  Timer1.initialize(kTimerPeriodUSec);
  Timer1.attachInterrupt(TimerInterruptHandler);
  attachInterrupt(kPinEcho0, Echo0InterruptHandler, CHANGE);
  attachInterrupt(kPinEcho1, Echo1InterruptHandler, CHANGE);
  attachInterrupt(kPinEcho2, Echo2InterruptHandler, CHANGE);
  attachInterrupt(kPinEcho3, Echo3InterruptHandler, CHANGE);
}

void TSonar::TimerInterruptHandler() {
  typedef enum { COUNTDOWN, PULSE_HIGH, PULSE_LOW } TTimerState;

  static volatile TTimerState state = COUNTDOWN;
  static volatile long countdown = kTimerCountsPerSamplingPeriod;

  if (--countdown == 0) {
    state = PULSE_HIGH;
    countdown = kTimerCountsPerSamplingPeriod;
  }

  switch (state) {
    case COUNTDOWN:
      break;

    case PULSE_HIGH:
      if ((g_next_sensor_index_ % 4) == 0) {
        digitalWrite(kPinTrigger0, HIGH);
      } else if ((g_next_sensor_index_ % 4) == 1) {
        digitalWrite(kPinTrigger1, HIGH);
      } else if ((g_next_sensor_index_ % 4) == 2) {
        digitalWrite(kPinTrigger2, HIGH);
      } else {
        digitalWrite(kPinTrigger3, HIGH);
      }

      state = PULSE_LOW;
      break;

    case PULSE_LOW:
      if ((g_next_sensor_index_ % 4) == 0) {
        digitalWrite(kPinTrigger0, LOW);
      } else if ((g_next_sensor_index_ % 4) == 1) {
        digitalWrite(kPinTrigger1, LOW);
      } else if ((g_next_sensor_index_ % 4) == 2) {
        digitalWrite(kPinTrigger2, LOW);
      } else {
        digitalWrite(kPinTrigger3, LOW);
      }

      g_next_sensor_index_++;
      state = COUNTDOWN;
      break;
  }
}

TSonar::TSonar() : TModule(TModule::kSonar) {
  for (size_t i = 0; i < kNumberSonars; i++) {
    for (size_t j = 0; j < kNumberReadingsToAverage; j++) {
      g_values_mm_history_[i][j] = 0;
    }
  }
}

TSonar& TSonar::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TSonar();
  }

  return *g_singleton_;
}

uint8_t TSonar::g_next_sensor_index_ = 0;

TSonar* TSonar::g_singleton_ = nullptr;

int TSonar::g_values_mm_[TSonar::kNumberSonars] = {-1, -1, -1, -1};

int TSonar::g_values_mm_history_[TSonar::kNumberSonars]
                                [TSonar::kNumberReadingsToAverage];

float TSonar::g_average_value_m_[TSonar::kNumberSonars] = {0, 0, 0, 0};

const float TSonar::g_time_to_mm_scaler_ = (10.0 / 2.0) / 29.1;