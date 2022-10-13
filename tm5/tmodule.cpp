#include "tmodule.h"

#include <Arduino.h>
#include <stdint.h>
#define DO_TIMING true

TModule::TModule() {
  if (g_nextModuleNumber < NUMBER_MODULES) {
    g_allModules[g_nextModuleNumber++] = this;
  }
}

void TModule::getStatistics(char* outString, size_t outStringSize) {
  static uint32_t statTimingStart = micros();
  // StringBuffer
  char statList[2048];

  statList[0] = '\0';

  for (int i = 0; i < g_nextModuleNumber; i++) {
    static size_t MAXLEN = 512;
    char temp[MAXLEN];
    temp[0] = '\0';
    snprintf(temp, MAXLEN, "%s min: %3.1f, max: %3.1f, avg: %3.1f\n",
             g_allModules[i]->name(), g_readings[i][MIN], g_readings[i][MAX],
             g_readings[i][SUM] / g_nextReadingNumber);
    strcat(statList, temp);
  }

  snprintf(outString, outStringSize,
           "--- --- --- duration for %d readings: %5.1f ms\n%s",
           g_nextReadingNumber, ((micros() * 1.0) - statTimingStart) / 1000.0,
           statList);
  resetReadings();
  g_nextReadingNumber = 0;
  statTimingStart = micros();
}

void TModule::doLoop() {
  for (int i = 0; i < g_nextModuleNumber; i++) {
    uint32_t start = micros();
    g_allModules[i]->loop();

    float duration = ((micros() * 1.0) - start) / 1000.0;
    g_readings[i][SUM] += duration;
    if (duration < g_readings[i][MIN]) {
      g_readings[i][MIN] = duration;
    }

    if (duration > g_readings[i][MAX]) {
      g_readings[i][MAX] = duration;
    }
  }

  g_nextReadingNumber++;
}

void TModule::resetReadings() {
  for (int i = 0; i < NUMBER_MODULES; i++) {
    g_readings[i][MIN] = 10'000'000;
    g_readings[i][MAX] = -10'000'000;
    g_readings[i][SUM] = 0;
  }
}

void TModule::doSetup() {
  for (int i = 0; i < g_nextModuleNumber; i++) {
    g_allModules[i]->setup();
  }
}

// TModule& TModule::singleton() {
//   if (!g_singleton) {
//     g_singleton = new TModule();
//     resetReadings();
//   }

//   return *g_singleton;
// }

TModule* TModule::g_allModules[TModule::NUMBER_MODULES + 1];
uint8_t TModule::g_nextModuleNumber = 0;
int TModule::g_nextReadingNumber = 0;
// TModule* TModule::g_singleton = nullptr;
float TModule::g_readings[TModule::NUMBER_MODULES][TModule::NUMBER_SLOTS];
