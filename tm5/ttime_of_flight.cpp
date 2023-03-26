#include "ttime_of_flight.h"

#include <VL53L0X.h>
#include <Wire.h>
#include <stdint.h>

#include "tconfiguration.h"
#include "tmicro_ros.h"

void TTimeOfFlight::UpdateSensorValue(TimeOfFlightEnum device) {
  if (device >= kNumberTimeOfFlightDevices) {
    // Invalid device number.
    return;
  }

  if (g_sensor_[device] == nullptr) {
    // Device failed to initialize.
    return;
  }

  SelectTimeOfFlightSensor(device);

  // Read the device if a new reading is ready.
  if ((g_sensor_[device]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) !=
      0) {
    {
      // Read the device range value(mm).
      g_cached_value_mm_[device] =
          g_sensor_[device]->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);

      // Clear the interrupt.
      g_sensor_[device]->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

      // Insert new reading into the round-robin queue.
      g_valuesMmHistory_[device][g_valuesMmHistoryIndex_[device]] =
          g_cached_value_mm_[device];
      g_valuesMmHistoryIndex_[device] += 1;
      if (g_valuesMmHistoryIndex_[device] >= kNumberReadingsToAverage) {
        g_valuesMmHistoryIndex_[device] = 0;
      }
    }
  }
}

float TTimeOfFlight::GetAverageValueM(TimeOfFlightEnum device) {
  // Update the queue with a new reading, if available.
  UpdateSensorValue(device);

  // Compute the average distance
  float averageSum_mm = 0.0;
  for (size_t i = 0; i < kNumberReadingsToAverage; i++) {
    averageSum_mm += (float)g_valuesMmHistory_[device][i];
  }

  return (averageSum_mm / (kNumberReadingsToAverage * 1.0)) * 0.001;
}

void TTimeOfFlight::loop() {
  for (uint8_t device = 0; device < kNumberTimeOfFlightDevices; device++) {
    TMicroRos::singleton().PublishTof(
        device, GetAverageValueM((TimeOfFlightEnum)device));
  }
}

void TTimeOfFlight::SelectTimeOfFlightSensor(TimeOfFlightEnum device) {
  if (device >= kNumberTimeOfFlightDevices) return;

  Wire.beginTransmission(kI2cMultiplexerAddress);
  Wire.write(1 << device);
  Wire.endTransmission();
}

void TTimeOfFlight::setup() {
  char diagnostic_message[128];
  if (TM5::kDoDetailDebug) {
    TMicroRos::singleton().PublishDiagnostic("INFO [TTimeOfFlight::setup]");
  }

  for (uint8_t device = 0; device < kNumberTimeOfFlightDevices; device++) {
    int retry_count = 0;
    bool success = false;
    while (!success && (++retry_count <= 4)) {
      SelectTimeOfFlightSensor(static_cast<TimeOfFlightEnum>(device));
      VL53L0X *sensor = new VL53L0X();
      sensor->setTimeout(500);
      if (sensor->init()) {
        g_sensor_[device] = sensor;
        sensor->setMeasurementTimingBudget(kTimingBudgetMs * 1000);
        sensor->startContinuous();
        snprintf(diagnostic_message, sizeof(diagnostic_message),
                 "INFO [TTimeOfFlight::setup] success device: %d", device);
        TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
        success = true;
      }
    }
    if (!success) {
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "ERROR [TTimeOfFlight::setup] FAIL device: %d", device);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      g_sensor_[device] = nullptr;
    }
  }
}

TTimeOfFlight::TTimeOfFlight() : TModule(TModule::kTimeOfFlight) {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  for (size_t i = 0; i < kNumberTimeOfFlightDevices; i++) {
    g_cached_value_mm_[i] = -1;
    g_sensor_[i] = nullptr;
  }

  for (size_t i = 0; i < kNumberTimeOfFlightDevices; i++) {
    for (size_t j = 0; j < kNumberReadingsToAverage; j++) {
      g_valuesMmHistory_[i][j] = 0.0;
    }

    g_valuesMmHistoryIndex_[i] = 0;
  }
}

TTimeOfFlight &TTimeOfFlight::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TTimeOfFlight();
  }

  return *g_singleton_;
}

TTimeOfFlight *TTimeOfFlight::g_singleton_ = nullptr;

int TTimeOfFlight::g_cached_value_mm_
    [TTimeOfFlight::kNumberTimeOfFlightDevices];
VL53L0X *TTimeOfFlight::g_sensor_[TTimeOfFlight::kNumberTimeOfFlightDevices];

int TTimeOfFlight::g_valuesMmHistory_[TTimeOfFlight::kNumberTimeOfFlightDevices]
                                     [TTimeOfFlight::kNumberReadingsToAverage];

int TTimeOfFlight::g_valuesMmHistoryIndex_[kNumberTimeOfFlightDevices];