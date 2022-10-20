#include "ttime_of_flight.h"

#include <VL53L0X.h>
#include <Wire.h>
#include <stdint.h>

#include "talert.h"
#include "tmicro_ros.h"

int TTimeOfFlight::GetValueMm(TimeOfFlightEnum device) {
  if (device >= kNumberTimeOfFlightDevices) {
    return -1;
  }

  SelectTimeOfFlightSensor(device);
  if ((g_sensor_[device]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) !=
      0) {
    {
      g_cached_value_mm_[device] =
          g_sensor_[device]->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
      g_sensor_[device]->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
      TMicroRos::publishTof((uint8_t)device, g_cached_value_mm_[device] * 0.001);
    }
  }

  if (g_sensor_[device]) {
    return g_cached_value_mm_[device];
  } else {
    return -1;
  }
}

void TTimeOfFlight::loop() {
  // static const TAlert::TAlertSource map[] = {
  //     TAlert::TOF_UPPER_LEFT_FORWARD,  TAlert::TOF_UPPER_RIGHT_FORWARD,
  //     TAlert::TOF_UPPER_LEFT_SIDEWAY,  TAlert::TOF_UPPER_RIGHT_SIDEWAY,
  //     TAlert::TOF_LOWER_LEFT_SIDEWAY,  TAlert::TOF_LOWER_RIGHT_SIDEWAY,
  //     TAlert::TOF_LOWER_LEFT_BACKWARD, TAlert::TOF_LOWER_RIGHT_BACKWARD};

  static uint8_t next_sensor_to_read = 0;

  (void)GetValueMm(static_cast<TimeOfFlightEnum>(next_sensor_to_read++));
  if (next_sensor_to_read >= kNumberTimeOfFlightDevices) {
    next_sensor_to_read = 0;
  }
  // for (uint8_t i = 0; i < NUMBER_TIME_OF_FLIGHT/2; i++) {
  //   int mm = getValueMm(static_cast<TIMEOFFLIGHT>(i));
  //   if (kDoStopMotorsOnCollisionThreat && (mm != -1) &&
  //       (mm < kAlertDistanceMm)) {
  //     // TAlert::singleton().set(map[i]);
  //   } else {
  //     // TAlert::singleton().reset(map[i]);
  //   }
  // }
}

void TTimeOfFlight::SelectTimeOfFlightSensor(TimeOfFlightEnum device) {
  if (device >= kNumberTimeOfFlightDevices) return;

  Wire.beginTransmission(kI2cMultiplexerAddress);
  Wire.write(1 << device);
  Wire.endTransmission();
}

void TTimeOfFlight::setup() {
  uint8_t numberSensorsFound = 0;
  for (uint8_t i = 0; i < kNumberTimeOfFlightDevices; i++) {
    SelectTimeOfFlightSensor(static_cast<TimeOfFlightEnum>(i));
    VL53L0X *sensor = new VL53L0X();
    sensor->setTimeout(500);
    if (sensor->init()) {
      g_sensor_[i] = sensor;
      numberSensorsFound++;
      sensor->setMeasurementTimingBudget(kTimingBudgetMs * 1000);
      sensor->startContinuous();
    } else {
      g_sensor_[i] = nullptr;
    }
  }
}

TTimeOfFlight::TTimeOfFlight() : TModule(TModule::kTIME_OF_FLIGHT) {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  for (int i = 0; i < kNumberTimeOfFlightDevices; i++) {
    g_cached_value_mm_[i] = -1;
  }
}

TTimeOfFlight &TTimeOfFlight::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TTimeOfFlight();
  }

  return *g_singleton_;
}

TTimeOfFlight *TTimeOfFlight::g_singleton_ = nullptr;

int TTimeOfFlight::g_cached_value_mm_[TTimeOfFlight::kNumberTimeOfFlightDevices];
VL53L0X *TTimeOfFlight::g_sensor_[TTimeOfFlight::kNumberTimeOfFlightDevices];
