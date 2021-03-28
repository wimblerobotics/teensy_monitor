#pragma once

#include <stdint.h>

#include <Wire.h>

class TSonar {
 public:
  static const uint8_t NUMBER_SENSORS = 4;

  static TSonar& singleton();

  // A value of < 0 => no sensor at that index;
  int getValueMm(uint8_t index);

  void loop();

  void setup();

 private:
  TSonar();

  static void echo0InterruptHandler();
  static void echo1InterruptHandler();
  static void echo2InterruptHandler();
  static void echo3InterruptHandler();
  static void timerInterruptHandler();

  static uint8_t g_nextSensorIndex;

  static TSonar* g_singleton;

  static int g_valuesMm[NUMBER_SENSORS];

  static const uint8_t PIN_ECHO0 = 35;
  static const uint8_t PIN_TRIGGER0 = 34;
  static const uint8_t PIN_ECHO1 = 37;
  static const uint8_t PIN_TRIGGER1 = 36;
  static const uint8_t PIN_ECHO2 = 15;
  static const uint8_t PIN_TRIGGER2 = 14;
  static const uint8_t PIN_ECHO3 = 41;
  static const uint8_t PIN_TRIGGER3 = 40;

  static const uint16_t TIMER_PERIOD_USEC = 20;
  static const uint16_t TIMER_SAMPLING_PERIOD_MSEC = 10;
  static const uint16_t TIMER_COUNTS_PER_SAMPLING_PERIOD =
      (TIMER_SAMPLING_PERIOD_MSEC * 1000) / TIMER_PERIOD_USEC;
};
