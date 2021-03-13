#pragma once

#include <stdint.h>
#include <Adafruit_INA260.h>

class TMotorCurrent {
public:

  static TMotorCurrent& singleton();

  int getValueMa(uint8_t index);

  void loop();

  void setup();

private:

  typedef enum {
    LEFT_MOTOR__ADDRESS = 0x40,
    RIGHT_MOTOR_ADDRESS = 0x41
  } MOTOR_PINS;

  TMotorCurrent();

  static TMotorCurrent* g_singleton;

  static const uint8_t AVERAGE_COUNT = 20;

  static Adafruit_INA260 g_leftINA260;
  static Adafruit_INA260 g_rightINA260;

  static bool g_haveLeftMotorSensor;
  static bool g_haveRightMotorSensor;
  
  static float g_leftMotorCurrentMa;
  static float g_rightMotorCurrentMa;

  static uint8_t g_nextMotorCurrentIndex;
  static float g_leftMotorCurrentMaReadings[AVERAGE_COUNT];
  static float g_rightMotorCurrentMaReadings[AVERAGE_COUNT];

};