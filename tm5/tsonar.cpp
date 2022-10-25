#include "tsonar.h"

#include <stdint.h>

#include "Arduino.h"
#include "TimerOne.h"
#include "Wire.h"
#include "talert.h"
#include "tmicro_ros.h"

void TSonar::commonInterruptHandler(uint8_t PIN, long& endTime, long& startTime,
                                    uint8_t& averageIndex, size_t SONAR_INDEX) {
  switch (digitalRead(PIN)) {
    case HIGH:
      endTime = 0;
      startTime = micros();
      break;

    case LOW:
      endTime = micros();
      g_valuesMm[SONAR_INDEX] = (endTime - startTime) * g_TimeToMmScaler;
      g_valuesMmHistory[SONAR_INDEX][averageIndex++] = g_valuesMm[SONAR_INDEX];
      if (averageIndex >= NUMBER_READINGS_TO_AVERAGE) {
        averageIndex = 0;
      }

      int averageSum = 0;
      for (size_t i = 0; i < NUMBER_READINGS_TO_AVERAGE; i++) {
        averageSum += g_valuesMmHistory[SONAR_INDEX][i];
      }

      g_averageValueM[SONAR_INDEX] = (averageSum * 0.001) / NUMBER_READINGS_TO_AVERAGE;

      TMicroRos::publishSonar(SONAR_INDEX, g_averageValueM[SONAR_INDEX]);
      break;
  }
}

float TSonar::getAverageValueM(SONAR device) {
  return g_averageValueM[device];
}

void TSonar::echo0InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  static uint8_t averageIndex = 0;
  commonInterruptHandler(PIN_ECHO0, endTime, startTime, averageIndex, 0);
}

void TSonar::echo1InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  static uint8_t averageIndex = 0;
  commonInterruptHandler(PIN_ECHO1, endTime, startTime, averageIndex, 1);
}

void TSonar::echo2InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  static uint8_t averageIndex = 0;
  commonInterruptHandler(PIN_ECHO2, endTime, startTime, averageIndex, 2);
}

void TSonar::echo3InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  static uint8_t averageIndex = 0;
  commonInterruptHandler(PIN_ECHO3, endTime, startTime, averageIndex, 3);
}

int TSonar::getValueMm(SONAR device) {
  if (static_cast<int>(device) >= NUMBER_SONARS) {
    return -1;
  } else {
    return g_valuesMm[static_cast<int>(device)];
  }
}

void TSonar::loop() {
  const int ALERT_DISTANCE_MM = 3 * 25.4;
  // TAlert::TAlertSource map[] = {TAlert::SONAR_FRONT, TAlert::SONAR_RIGHT,
  //                               TAlert::SONAR_BACK, TAlert::SONAR_LEFT};

  for (uint8_t i = 0; i < NUMBER_SONARS; i++) {
    if (doStopMotorsOnCollisionThreat &&
        (getValueMm(static_cast<SONAR>(i)) < ALERT_DISTANCE_MM)) {
      // TAlert::singleton().set(map[i]);
    } else {
      // TAlert::singleton().reset(map[i]);
    }
  }
}

void TSonar::setup() {
  pinMode(PIN_ECHO0, INPUT);
  pinMode(PIN_TRIGGER0, OUTPUT);
  pinMode(PIN_ECHO1, INPUT);
  pinMode(PIN_TRIGGER1, OUTPUT);
  pinMode(PIN_ECHO2, INPUT);
  pinMode(PIN_TRIGGER2, OUTPUT);
  pinMode(PIN_ECHO3, INPUT);
  pinMode(PIN_TRIGGER3, OUTPUT);
  Timer1.initialize(TIMER_PERIOD_USEC);
  Timer1.attachInterrupt(timerInterruptHandler);
  attachInterrupt(PIN_ECHO0, echo0InterruptHandler, CHANGE);
  attachInterrupt(PIN_ECHO1, echo1InterruptHandler, CHANGE);
  attachInterrupt(PIN_ECHO2, echo2InterruptHandler, CHANGE);
  attachInterrupt(PIN_ECHO3, echo3InterruptHandler, CHANGE);
}

void TSonar::timerInterruptHandler() {
  typedef enum { COUNTDOWN, PULSE_HIGH, PULSE_LOW } TTimerState;

  static volatile TTimerState state = COUNTDOWN;
  static volatile long countdown = TIMER_COUNTS_PER_SAMPLING_PERIOD;

  if (--countdown == 0) {
    state = PULSE_HIGH;
    countdown = TIMER_COUNTS_PER_SAMPLING_PERIOD;
  }

  switch (state) {
    case COUNTDOWN:
      break;

    case PULSE_HIGH:
      if ((g_nextSensorIndex % 4) == 0) {
        digitalWrite(PIN_TRIGGER0, HIGH);
      } else if ((g_nextSensorIndex % 4) == 1) {
        digitalWrite(PIN_TRIGGER1, HIGH);
      } else if ((g_nextSensorIndex % 4) == 2) {
        digitalWrite(PIN_TRIGGER2, HIGH);
      } else {
        digitalWrite(PIN_TRIGGER3, HIGH);
      }

      state = PULSE_LOW;
      break;

    case PULSE_LOW:
      if ((g_nextSensorIndex % 4) == 0) {
        digitalWrite(PIN_TRIGGER0, LOW);
      } else if ((g_nextSensorIndex % 4) == 1) {
        digitalWrite(PIN_TRIGGER1, LOW);
      } else if ((g_nextSensorIndex % 4) == 2) {
        digitalWrite(PIN_TRIGGER2, LOW);
      } else {
        digitalWrite(PIN_TRIGGER3, LOW);
      }

      g_nextSensorIndex++;
      state = COUNTDOWN;
      break;
  }
}

TSonar::TSonar() : TModule(TModule::kSONAR) {
  for (size_t i = 0; i < NUMBER_SONARS; i++) {
    for (size_t j = 0; j < NUMBER_READINGS_TO_AVERAGE; j++) {
      g_valuesMmHistory[i][j] = 0;
    }
  }
}

TSonar& TSonar::singleton() {
  if (!g_singleton) {
    g_singleton = new TSonar();
  }

  return *g_singleton;
}

uint8_t TSonar::g_nextSensorIndex = 0;

TSonar* TSonar::g_singleton = nullptr;

int TSonar::g_valuesMm[TSonar::NUMBER_SONARS] = {-1, -1, -1, -1};

int TSonar::g_valuesMmHistory[TSonar::NUMBER_SONARS]
                             [TSonar::NUMBER_READINGS_TO_AVERAGE];

float TSonar::g_averageValueM[TSonar::NUMBER_SONARS] = {0, 0, 0, 0};

const float TSonar::g_TimeToMmScaler = (10.0 / 2.0) / 29.1;