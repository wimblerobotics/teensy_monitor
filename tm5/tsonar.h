#pragma once

#include <Wire.h>
#include <stdint.h>

#include "tmodule.h"

class TSonar : TModule {
 public:
  // Which SONAR sensor.
  typedef enum Sonar {
    kFront,
    kRight,
    kBack,
    kLeft,
    kNumberSonars  // Number of SONAR sensors.
  } Sonar;

  // Get sensed range for the device. A value of < 0 => no sensor 'device'
  // detected during setup.
  int GetValueMm(Sonar device);

  // Get the average range for the device.
  float GetAverageValueM(Sonar device);

  // Singleton constructor.
  static TSonar& singleton();

 protected:
  // From TModule.‰
  void loop();

  // From TModule.‰
  const char* name() { return "Sonr"; }

  // From TModule.‰
  void setup();

 private:
  // Number of readings to average together to reduce noise.
  static const uint8_t kNumberReadingsToAverage = 4;

  // Private constructor.
  TSonar();

  // Common interrupt handler.
  static void CommonInterruptHandler(uint8_t pin, long& end_time,
                                     long& start_time, uint8_t& average_index,
                                     size_t sonar_index);

  // Interrupt handler for per device echo.
  static void Echo0InterruptHandler();
  static void Echo1InterruptHandler();
  static void Echo2InterruptHandler();
  static void Echo3InterruptHandler();
  static void TimerInterruptHandler();

  // Next device being handled in the round-robin processing.
  static uint8_t g_next_sensor_index_;

  // Singleton instance.
  static TSonar* g_singleton_;

  // Last captured sensed distance for each SONAR device.
  static int g_values_mm_[kNumberSonars];

  // List of last values, for computing an average.
  static int g_values_mm_history_[kNumberSonars][kNumberReadingsToAverage];

  // Last captured average distance for each SONAR device.
  static float g_average_value_m_[kNumberSonars];

  // GPIO pins for controlling the SONAR sensors.
  enum {
    kPinEcho0 = 35,
    kPinTrigger0 = 34,
    kPinEcho1 = 37,
    kPinTrigger1 = 36,
    kPinEcho2 = 41,
    kPinTrigger2 = 40,
    kPinEcho3 = 15,
    kPinTrigger3 = 14
  };

  // Microseconds per timer interrupt.
  static const uint16_t kTimerPeriodUSec = 20;

  // Desired milliseconds to wait between a low pulse and the next high pulse.
  static const uint16_t kTimerSamplingPeriodMSec = 10;

  // Timer interrupts to expire before a low pulse to high pulse transition is
  // taken..
  static const uint16_t kTimerCountsPerSamplingPeriod =
      (kTimerSamplingPeriodMSec * 1000) / kTimerPeriodUSec;

  // For converting SONAR echo time to millimeters.
  static const float g_time_to_mm_scaler_;
};
