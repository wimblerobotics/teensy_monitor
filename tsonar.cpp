#include "tsonar.h"

#include <stdint.h>

#include "Arduino.h"
#include "talert.h"
#include "TimerOne.h"
#include "Wire.h"

TSonar::TSonar() {}

TSonar& TSonar::singleton() {
  if (!g_singleton) {
    g_singleton = new TSonar();
  }

  return *g_singleton;
}


void TSonar::echo0InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  switch (digitalRead(PIN_ECHO0)) {
    case HIGH:
      endTime = 0;
      startTime = micros();
      break;

    case LOW:
      endTime = micros();
      g_valuesMm[0] = ((endTime - startTime) * 10 / 2) / 29.1;
      break;
  }
}


void TSonar::echo1InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  switch (digitalRead(PIN_ECHO1)) {
    case HIGH:
      endTime = 0;
      startTime = micros();
      break;

    case LOW:
      endTime = micros();
      g_valuesMm[1] = ((endTime - startTime) * 10 / 2) / 29.1;
      break;
  }
}


int TSonar::getValueMm(uint8_t index) {
  if (index >= NUMBER_SENSORS) {
    return -1;
  } else {
    return g_valuesMm[index];
  }
}


void TSonar::loop() {
  const int ALERT_DISTANCE_MM = 3 * 25.4;
  TAlert::TAlertSource map[] = {
    TAlert::SONAR_FRONT,
    TAlert::SONAR_RIGHT,
    TAlert::SONAR_BACK,
    TAlert::SONAR_LEFT
  };

  for (uint8_t i = 0; i < 1/*NUMBER_SENSORS*/; i++) {
    if (getValueMm(i) < ALERT_DISTANCE_MM) {
      Serial.print("Sonar dist: ");Serial.println(getValueMm(i));
      TAlert::singleton().set(map[i]);
    } else {
      TAlert::singleton().unset(map[i]);
    }
  }
}


void TSonar::setup() {
  pinMode(PIN_ECHO0, INPUT);
  pinMode(PIN_TRIGGER0, OUTPUT);
  pinMode(PIN_ECHO1, INPUT);
  pinMode(PIN_TRIGGER1, OUTPUT);
  Timer1.initialize(TIMER_PERIOD_USEC);
  Timer1.attachInterrupt(timerInterruptHandler);
  attachInterrupt(PIN_ECHO0, echo0InterruptHandler, CHANGE);
  attachInterrupt(PIN_ECHO1, echo1InterruptHandler, CHANGE);
}


void TSonar::timerInterruptHandler() {
  typedef enum { COUNTDOWN, PULSE_HIGH, PULSE_LOW } STATE;

  static volatile STATE state = COUNTDOWN;
  static volatile long countdown = TIMER_COUNTS_PER_SAMPLING_PERIOD;

  if (--countdown == 0) {
    state = PULSE_HIGH;
    countdown = TIMER_COUNTS_PER_SAMPLING_PERIOD;
  }

  switch (state) {
    case COUNTDOWN:
      break;

    case PULSE_HIGH:
      if ((g_nextSensorIndex % 2) == 0) {
        digitalWrite(PIN_TRIGGER0, HIGH);
      } else {
        digitalWrite(PIN_TRIGGER1, HIGH);
      }

      state = PULSE_LOW;
      break;

    case PULSE_LOW:
      if ((g_nextSensorIndex % 2) == 0) {
        digitalWrite(PIN_TRIGGER0, LOW);
      } else {
        digitalWrite(PIN_TRIGGER1, LOW);
      }

      g_nextSensorIndex++;
      state = COUNTDOWN;
      break;
  }
}


uint8_t TSonar::g_nextSensorIndex = 0;

TSonar* TSonar::g_singleton = nullptr;

int TSonar::g_valuesMm[TSonar::NUMBER_SENSORS] = {-1, -1, -1, -1};