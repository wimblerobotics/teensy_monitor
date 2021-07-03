#pragma once

#include <stdint.h>

#include "tmodule.h"

class TRelay : TModule {
 public:
  typedef enum {
    INTEL_POWER,
    INTEL_RESET,
    MOTOR_POWER,
    NVIDIA_POWER,
    NVIDIA_RESET,
    NUMBER_DEVICES
  } TRelayDevice;

  bool isSet(TRelayDevice device);

  void loop();

  const char* name() { return "TRelay"; }
  
  void powerOn(TRelayDevice device);

  void setup();

  static TRelay& singleton();

  void powerOff(TRelayDevice device);

 private:
  typedef enum {
    INTEL_ON_OFF_PIN = 0,
    INTEL_RESET_PIN = 1,
    MOTOR_ON_OFF_PIN = 2,
    NVIDIA_ON_OFF_PIN = 3,
    NVIDIA_RESET_PIN = 4
  } TPinNumbers;

  static const uint32_t RESET_DURATION_MS = 1000;

  TRelay();

  static uint32_t g_deviceSetTimeMs[NUMBER_DEVICES];

  static TRelay* g_singleton;
};