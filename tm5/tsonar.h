#pragma once

#include <Wire.h>
#include <stdint.h>

#include "tmodule.h"

class TSonar : TModule {
 public:
  // Which SONAR sensor.
  typedef enum SONAR {
    FRONT,
    RIGHT,
    BACK,
    LEFT,
    NUMBER_SONARS  // Number of SONAR sensors.
  } SONAR;

  // Get sensed range for device. A value of < 0 => no sensor 'device' detected
  // during setup.
  int getValueMm(SONAR device);

  float getAverageValueM(SONAR device);

  // Singleton constructor.
  static TSonar& singleton();

 protected:
  // From TModule.‰
  void loop();

  // From TModule.‰
  const char* name() { return "SO"; }

  // From TModule.‰
  void setup();

 private:
  // Number of readings to average.
  enum MISC_CONSTANTS { NUMBER_READINGS_TO_AVERAGE = 4 };

  // Should motors be put in e-stop if collision is imminent?
  static const bool doStopMotorsOnCollisionThreat = false;

  // Private constructor.
  TSonar();

  // Common interrupt handler.
  static void commonInterruptHandler(uint8_t PIN, long& endTime,
                                     long& startTime, uint8_t& averageIndex,
                                     size_t SONAR_INDEX);

  // Interrupt handler for device echo..
  static void echo0InterruptHandler();
  static void echo1InterruptHandler();
  static void echo2InterruptHandler();
  static void echo3InterruptHandler();
  static void timerInterruptHandler();

  // Next device being handled in the round-robin processing.
  static uint8_t g_nextSensorIndex;

  // Singleton instance.
  static TSonar* g_singleton;

  // Last captured sensed distance for each SONAR device.
  static int g_valuesMm[NUMBER_SONARS];

  static int g_valuesMmHistory[NUMBER_SONARS][NUMBER_READINGS_TO_AVERAGE];

  static float g_averageValueM[NUMBER_SONARS];

  // Minimum detection distance before an alert is raised
  static const int ALERT_DISTANCE_MM = 3 * 25.4;

  // GPIO pins for controlling the SONAR sensors.
  enum {
    PIN_ECHO0 = 35,
    PIN_TRIGGER0 = 34,
    PIN_ECHO1 = 37,
    PIN_TRIGGER1 = 36,
    PIN_ECHO2 = 41,
    PIN_TRIGGER2 = 40,
    PIN_ECHO3 = 15,
    PIN_TRIGGER3 = 14
  };

  // Microseconds per timer interrupt.
  static const uint16_t TIMER_PERIOD_USEC = 20;

  // Desired milliseconds to wait between a low pulse and the next high pulse.
  static const uint16_t TIMER_SAMPLING_PERIOD_MSEC = 10;

  // Timer interrupts to expire before a low pulse to high pulse transition is
  // taken..
  static const uint16_t TIMER_COUNTS_PER_SAMPLING_PERIOD =
      (TIMER_SAMPLING_PERIOD_MSEC * 1000) / TIMER_PERIOD_USEC;

  // For converting SONAR echo time to millimeters.
  static const float g_TimeToMmScaler;
};
