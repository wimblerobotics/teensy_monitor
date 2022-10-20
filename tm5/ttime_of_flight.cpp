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
      TMicroRos::publishTof((uint8_t)device,
                            g_cachedValue[device] * 0.001);
    }
  }

  // if ((g_sensor[device]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07)
  // !=
  //     0) {
  //   g_cachedValue[device] =
  //   g_sensor[device]->readRangeContinuousMillimeters();
  // }

  // // Compute the minimum time between device requests to read the distance.
  // // This is found by dividing the device timing budget by the number of
  // active
  // // sensors.
  // uint8_t number_active_sensors = 0;
  // for (size_t i = 0; i < NUMBER_TIME_OF_FLIGHT; i++) {
  //   if (g_sensor[i] != nullptr) {
  //     number_active_sensors++;
  //   }
  // }

  // uint16_t period_between_device_reads_ms =
  //     kTimingBudgetMs / number_active_sensors;

  // // Get the sensor values only once every give number of milli seconds.
  // static uint8_t lastSensedIndex = 0;
  // static unsigned long lastWallTime = millis();
  // unsigned long currentWallTime = millis();
  // unsigned long durationSinceLastSense = currentWallTime - lastWallTime;

  // if (durationSinceLastSense > period_between_device_reads_ms) {
  //   if (g_sensor[lastSensedIndex] != nullptr) {
  //
  //     g_cachedValue[lastSensedIndex] =
  //         g_sensor[lastSensedIndex]->readRangeContinuousMillimeters();
  //     TMicroRos::publishTof(lastSensedIndex,
  //                           g_cachedValue[lastSensedIndex] * 0.001);
  //   }

  //   lastSensedIndex += 1;
  //   if (lastSensedIndex >= NUMBER_TIME_OF_FLIGHT) {
  //     lastSensedIndex = 0;
  //   }

  //   lastWallTime = currentWallTime;
  // }

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

  int mm = getValueMm(static_cast<TIMEOFFLIGHT>(next_sensor_to_read++));
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
