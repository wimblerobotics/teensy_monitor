#pragma once

#include <VL53L0X.h>
#include <stdint.h>

#include "tmodule.h"

class TTimeOfFlight : TModule {
 public:
  typedef enum TimeOfFlightEnum {
    kUpperLeftForwards,
    kUpperRightForwards,
    kUpperLeftSideways,
    kUpperRightSideways,
    kLeftLeftSideways,
    kLowerRightSideways,
    kLowerLeftBackwards,
    kLowerRightBackwards,
    kNumberTimeOfFlightDevices  // Number of time-of-flight devices.
  } TimeOfFlightEnum;

  // Get the sensed distance for the device. A value of < 0 => no sensor at that
  // index.
  int GetValueMm(TimeOfFlightEnum device);

  // Singleton constructor.
  static TTimeOfFlight& singleton();

 protected:
  // From TModule.‰
  void loop();

  // From TModule.‰
  const char* name() { return "ToFl"; }

  // From TModule.‰
  void setup();

 private:
  // Number of readings to average.
  enum MiscConstantsEnum { kNumberReadingsToAverage = 4 };

  // Should motors be put in e-stop if collision is imminent?
  static const bool kDoStopMotorsOnCollisionThreat = false;

  // Private constructor.
  TTimeOfFlight();

  // Select a time-of-flight device through the multiplexer.
  void SelectTimeOfFlightSensor(TimeOfFlightEnum device);

  // Address of I2C multiplexer for time-of-flight devices.
  static const uint8_t kI2cMultiplexerAddress = 0x70;

  // Last sensed distance for each time-of-flight device.
  static int g_cached_value_mm_[kNumberTimeOfFlightDevices];

  // Round-robin  history of last value readings per device.
  static int g_valuesMmHistory_[kNumberTimeOfFlightDevices][kNumberReadingsToAverage];

// Round-robin index into g_values_mm_history_ per device.
  static int g_valuesMmHistoryIndex_[kNumberTimeOfFlightDevices];

  // Device hardware handle for each time-of-flight device.
  static VL53L0X* g_sensor_[kNumberTimeOfFlightDevices];

  // Singleton instance.
  static TTimeOfFlight* g_singleton_;

  static const int16_t kTimingBudgetMs = 33;
};
