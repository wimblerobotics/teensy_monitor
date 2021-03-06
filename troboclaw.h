/**
 * MIT License
 * Copyright 2021 by Michael Wimble
 * 
 * Interface to the RoboClaw controller.
 * 
 * This module alks over serial bus and gathers several values of 
 * interest, such as the motor encoder values, motor speeds, and 
 * motor currents. Also, a check is made of the software version.
 * 
 * If any operation fails, the serial port is dropped and restarted.
 * The average loop duration is 1.05 ms with a min of 0.54 ms and
 * a max of 3.17 ms.
 * 
 * To minimum loop duration, during each call to loop(), the next 
 * parameter is read from the RoboClaw via state machine, so each
 * loop() call results in one parameter being read. That means, for
 * an average loop duration 1.05 ms, and there being 8 parameters
 * read, the total duration to read all parameters is 8.4 ms on
 * average, or just under 12 frame updates per second.
 */

#pragma once

#include "RoboClaw.h"
#include "tmodule.h"

class TRoboClaw : TModule {
 public:

  // Get the logic battery voltage.
  float getBatteryLogic();

  // Get the main battery voltage.
  float getBatteryMain();

  // Get the motor current (amps) for motor 1. This is
  // always a positive number.
  float getM1Current();

  // Get the encoder counts for motor 1.
  int32_t getM1Encoder();

  // Get the speed of motor 1 in encoder pulses/second.
  int32_t getM1Speed();

  // Get the motor current (amps) for motor 2. This is
  // always a positive number.
  float getM2Current();

  // Get the encoder counts for motor 1.
  int32_t getM2Encoder();

  // Get the speed of motor 2 in encoder pulses/second.
  int32_t getM2Speed();

  // From TModule.
  void loop();

  // From TModule.
  virtual const char* name() { return "TRoboClaw"; }

  // From TModule.
  void setup();

  // Singleton constructor.
  static TRoboClaw& singleton();

  typedef enum TSTATE {
    CURRENTS,
    ENCODER_M1,
    ENCODER_M2,
    LOGIC_BATTERY,
    MAIN_BATTERY,
    SPEED_M1,
    SPEED_M2,
    VERSION
  } TSTATE;

 private:
  static const int DEVICE_ADDRESS = 0x80;

  static const char* DEVICE_VERSION;

  // Private constructor.
  TRoboClaw();

  // Get current for motor 1 and 2;
  void getCurrents();

  // Get encoder value for motor 1;
  void getEncoderM1();

  // Get encoder value for motor 2;
  void getEncoderM2();

  // Get logic battery voltage;
  void getLogicBattery();

  // Get main battery voltage;
  void getMainBattery();

  // Get speed for motor 1;
  void getSpeedM1();

  // Get speed for motor 2;
  void getSpeedM2();

  // Get device version string;
  void getVersion();

  // Reestablish connection to device.
  void reconnect();

  // Motor currents.
  int16_t g_current_m1;
  int16_t g_current_m2;

  // Motor encoders.
  int32_t g_encoder_m1;
  int32_t g_encoder_m2;
  
  // Battery voltages;
  int16_t g_logic_battery;
  int16_t g_main_battery;

  static RoboClaw g_roboclaw;

  // Singleton instance.
  static TRoboClaw* g_singleton;

  // Motor speeds.
  int32_t g_speed_m1;
  int32_t g_speed_m2;

  // State machine state.
  static TSTATE g_state;
};