#pragma once

#include <stdint.h>

// template <typename T>
class TModule {
 public:
  typedef enum MODULE {
    ALARM,
    ALERT,
    MOTOR_CURRENT,
    PANEL_SELECTOR,
    RELAY,
    ROS_CLIENT,
    SERVER,
    SONAR,
    TEMPERATURE,
    TIME_OF_FLIGHT,
    NUMBER_MODULES
  } MODULE;

  virtual void loop();
  virtual const char* name() { return "TModule"; }
  virtual void setup();
  static TModule& singleton();

 protected:
  TModule();

 private:
  typedef enum SLOT {
    MIN,
    MAX,
    SUM,
    NUMBER_SLOTS,
    NUMBER_READINGS = 1000
  } SLOT;

  static void resetReadings();

  static TModule* g_allModules[];
  static uint8_t g_nextModuleNumber;
  static int g_nextReadingNumber;
  static TModule* g_singleton;
  static float g_readings[NUMBER_MODULES][NUMBER_SLOTS];
};