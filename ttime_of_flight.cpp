#include "ttime_of_flight.h"

#include <stdint.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "talert.h"


TTimeOfFlight::TTimeOfFlight() {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
}


int TTimeOfFlight::getValueMm(uint8_t index) {
  if (index > 7) {
    return -1;
  }
  
  if (g_sensor[index]) {
    selectTimeOfFlightSensor(index);
    return g_sensor[index]->readRangeContinuousMillimeters();
  } else {
    return -1;
  }
}


void TTimeOfFlight::loop() {
  const int ALERT_DISTANCE_MM = 3 * 25.4;

  TAlert::TAlertSource map[] = {
    TAlert::TOF_UPPER_LEFT_FORWARD,
    TAlert::TOF_UPPER_RIGHT_FORWARD,
    TAlert::TOF_UPPER_LEFT_SIDEWAY,
    TAlert::TOF_UPPER_RIGHT_SIDEWAY,
    TAlert::TOF_LOWER_LEFT_SIDEWAY,
    TAlert::TOF_LOWER_RIGHT_SIDEWAY,
    TAlert::TOF_LOWER_LEFT_BACKWARD,
    TAlert::TOF_LOWER_RIGHT_BACKWARD
  };

  for (uint8_t i = 0; i < 1/*(NUMBER_SENSORS*/; i++) {
    if ((getValueMm(i) != -1) && (getValueMm(i) < ALERT_DISTANCE_MM)) {
    Serial.print("Dist: ");Serial.println(getValueMm(i)); //#####
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
      sensor->startContinuous();
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

VL53L0X* TTimeOfFlight::g_sensor[TTimeOfFlight::NUMBER_SENSORS];