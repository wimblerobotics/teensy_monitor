#pragma once

#include <stdint.h>
#include <VL53L0X.h>
#include <VL6180X.h>


class TTimeOfFlight {
 public:
  static const uint8_t NUMBER_SENSORS = 8;

  static TTimeOfFlight& singleton();

  // A value of < 0 => no sensor at that index.
  int getValueMm(uint8_t index);

  void loop();

  void setup();

 private:
  TTimeOfFlight();

  void selectTimeOfFlightSensor(uint8_t index);

  static const uint8_t I2C_MULTIPLEXER_ADDRESS = 0x70;

  static VL53L0X* g_sensor[NUMBER_SENSORS];
  static VL6180X* g_sensor2[NUMBER_SENSORS];

  static TTimeOfFlight* g_singleton;
};