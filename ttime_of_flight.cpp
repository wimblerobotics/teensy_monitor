#include "ttime_of_flight.h"

#include <stdint.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "talert.h"


TTimeOfFlight::TTimeOfFlight() {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  for (int i = 0; i < NUMBER_SENSORS; i++) {
    g_cachedValue[i] = 123;
  }
}


int TTimeOfFlight::getValueMm(uint8_t index) {
  if (index > 7) {
    return -1;
  }

  // Get the sensor values only once every give number of milli seconds.
  static uint8_t lastSensedIndex = 0;
  static const unsigned long GIVEN_NUMBER_OF_MILLISECONDS = 10;
  static unsigned long lastWallTime = millis();
  unsigned long currentWallTime = millis();
  unsigned long durationSinceLastSense = currentWallTime - lastWallTime;
//  Serial.print("currentWallTime: ");Serial.print(currentWallTime);
//  Serial.print(", lastWallTime: ");Serial.print(lastWallTime);
//  Serial.print(", GIVEN_NUMBER_OF_MILLISECONDS: ");Serial.print(GIVEN_NUMBER_OF_MILLISECONDS);
//  Serial.print(", durationSinceLastSense: ");Serial.print(durationSinceLastSense);
//  Serial.print(", lastSensedIndex: ");Serial.print(lastSensedIndex);
  if (durationSinceLastSense > GIVEN_NUMBER_OF_MILLISECONDS) {
    if (g_sensor[lastSensedIndex] != nullptr) {
      selectTimeOfFlightSensor(lastSensedIndex);
      g_cachedValue[lastSensedIndex] = g_sensor[lastSensedIndex]->readRangeContinuousMillimeters();
//      Serial.print("+");
    }
//    else { Serial.print("#"); }

//    Serial.print(lastSensedIndex);
//    Serial.print(" mm: ");Serial.println(g_cachedValue[lastSensedIndex]);
//    Serial.flush();

    lastSensedIndex += 1;
    if (lastSensedIndex >= NUMBER_SENSORS) {
      lastSensedIndex = 0;
    }

    lastWallTime = currentWallTime;
//    Serial.print(", g_cachedValue[lastSensedIndex]: ");Serial.print(g_cachedValue[lastSensedIndex]);
  }
//  Serial.println();
  
  if (g_sensor[index]) {
    return g_cachedValue[index];
  } else {
    return -1;
  }
}


void TTimeOfFlight::loop() {
//  Serial.print("TTOF-Loop, NUMBER_SENSORS: ");Serial.println(NUMBER_SENSORS);
  const int ALERT_DISTANCE_MM = 3 * 25.4;

  static const TAlert::TAlertSource map[] = {
    TAlert::TOF_UPPER_LEFT_FORWARD,
    TAlert::TOF_UPPER_RIGHT_FORWARD,
    TAlert::TOF_UPPER_LEFT_SIDEWAY,
    TAlert::TOF_UPPER_RIGHT_SIDEWAY,
    TAlert::TOF_LOWER_LEFT_SIDEWAY,
    TAlert::TOF_LOWER_RIGHT_SIDEWAY,
    TAlert::TOF_LOWER_LEFT_BACKWARD,
    TAlert::TOF_LOWER_RIGHT_BACKWARD
  };

  for (uint8_t i = 0; i < NUMBER_SENSORS; i++) {
//    Serial.flush();
//    uint32_t start = micros();
    int mm = getValueMm(i);
//    float duration = ((micros() * 1.0) - start) / 1000.0;
//    Serial.print("i: ");Serial.print(i);
//    Serial.print(", getValueMm duration (ms): ");Serial.println(duration);
//    Serial.print("i: ");Serial.print(i);Serial.print(",Dist: ");Serial.println(getValueMm(i)); //#####
    if ((mm != -1) && (mm < ALERT_DISTANCE_MM)) {
      TAlert::singleton().set(map[i]);
    } else {
      TAlert::singleton().unset(map[i]);
    }
  }
}


void TTimeOfFlight::selectTimeOfFlightSensor(uint8_t index) {
  if (index > 7) return;

  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << index);
  Wire.endTransmission();
}


void TTimeOfFlight::setup() {
  uint8_t numberSensorsFound = 0;
  for (uint8_t i =0; i < NUMBER_SENSORS; i++) {
    selectTimeOfFlightSensor(i);
    VL53L0X* sensor = new VL53L0X();
    sensor->setTimeout(500);
    if (sensor->init()) {
      g_sensor[i] = sensor;
      numberSensorsFound++;
      sensor->setMeasurementTimingBudget(20000);
      sensor->startContinuous();
    }
    else {
      g_sensor[i] = nullptr;
//      Serial.print("TTOF setup fail for i: ");Serial.println(i);//#####
    }
  }
}


TTimeOfFlight& TTimeOfFlight::singleton() {
  if (!g_singleton) {
    g_singleton = new TTimeOfFlight();
  }

  return *g_singleton;
}


TTimeOfFlight* TTimeOfFlight::g_singleton = nullptr;

int TTimeOfFlight::g_cachedValue[TTimeOfFlight::NUMBER_SENSORS];
VL53L0X* TTimeOfFlight::g_sensor[TTimeOfFlight::NUMBER_SENSORS];
