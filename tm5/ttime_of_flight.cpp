#include "ttime_of_flight.h"

#include <VL53L0X.h>
#include <Wire.h>
#include <stdint.h>

#include "talert.h"
#include "tmicro_ros.h"

int TTimeOfFlight::getValueMm(TIMEOFFLIGHT device) {
  if (device >= NUMBER_TIME_OF_FLIGHT) {
    return -1;
  }

  selectTimeOfFlightSensor(device);
  if ((g_sensor[device]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) !=
      0) {
    {
      g_cachedValue[device] =
          g_sensor[device]->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
      g_sensor[device]->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
      TMicroRos::publishTof((uint8_t)device, g_cachedValue[device] * 0.001);
    }
  }

  if (g_sensor[device]) {
    return g_cachedValue[device];
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

  (void)getValueMm(static_cast<TIMEOFFLIGHT>(next_sensor_to_read++));
  if (next_sensor_to_read >= NUMBER_TIME_OF_FLIGHT) {
    next_sensor_to_read = 0;
  }
  // for (uint8_t i = 0; i < NUMBER_TIME_OF_FLIGHT/2; i++) {
  //   int mm = getValueMm(static_cast<TIMEOFFLIGHT>(i));
  //   if (doStopMotorsOnCollisionThreat && (mm != -1) &&
  //       (mm < ALERT_DISTANCE_MM)) {
  //     // TAlert::singleton().set(map[i]);
  //   } else {
  //     // TAlert::singleton().reset(map[i]);
  //   }
  // }
}

void TTimeOfFlight::selectTimeOfFlightSensor(TIMEOFFLIGHT device) {
  if (device >= NUMBER_TIME_OF_FLIGHT) return;

  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << device);
  Wire.endTransmission();
}

void TTimeOfFlight::setup() {
  uint8_t numberSensorsFound = 0;
  for (uint8_t i = 0; i < NUMBER_TIME_OF_FLIGHT; i++) {
    selectTimeOfFlightSensor(static_cast<TIMEOFFLIGHT>(i));
    VL53L0X *sensor = new VL53L0X();
    sensor->setTimeout(500);
    if (sensor->init()) {
      g_sensor[i] = sensor;
      numberSensorsFound++;
      sensor->setMeasurementTimingBudget(kTimingBudgetMs * 1000);
      sensor->startContinuous();
    } else {
      g_sensor[i] = nullptr;
    }
  }
}

TTimeOfFlight::TTimeOfFlight() : TModule(TModule::kTIME_OF_FLIGHT) {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  for (int i = 0; i < NUMBER_TIME_OF_FLIGHT; i++) {
    g_cachedValue[i] = -1;
  }
}

TTimeOfFlight &TTimeOfFlight::singleton() {
  if (!g_singleton) {
    g_singleton = new TTimeOfFlight();
  }

  return *g_singleton;
}

TTimeOfFlight *TTimeOfFlight::g_singleton = nullptr;

int TTimeOfFlight::g_cachedValue[TTimeOfFlight::NUMBER_TIME_OF_FLIGHT];
VL53L0X *TTimeOfFlight::g_sensor[TTimeOfFlight::NUMBER_TIME_OF_FLIGHT];
