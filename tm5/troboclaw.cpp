#include "troboclaw.h"

#include <string.h>

#include "Arduino.h"
#include "RoboClaw.h"
#include "tmicro_ros.h"
#include "trelay.h"

#define HWSERIAL Serial6

void TRoboClaw::doMixedSpeedDist(int32_t m1_quad_pulses_per_second,
                                 int32_t m1_max_distance,
                                 int32_t m2_quad_pulses_per_second,
                                 int32_t m2_max_distance) {
  g_roboclaw.SpeedDistanceM1M2(DEVICE_ADDRESS, m1_quad_pulses_per_second,
                               m1_max_distance, m2_quad_pulses_per_second,
                               m2_max_distance, 1);
}

void TRoboClaw::doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                                      int32_t m1_quad_pulses_per_second,
                                      uint32_t m1_max_distance,
                                      int32_t m2_quad_pulses_per_second,
                                      uint32_t m2_max_distance) {
  g_roboclaw.SpeedAccelDistanceM1M2(
      DEVICE_ADDRESS, accel_quad_pulses_per_second, m1_quad_pulses_per_second,
      m1_max_distance, m2_quad_pulses_per_second, m2_max_distance, 1);
}

float TRoboClaw::getBatteryLogic() { return g_logic_battery / 10.0; }

float TRoboClaw::getBatteryMain() { return g_main_battery / 10.0; }

void TRoboClaw::getCurrents() {
  int16_t currentM1;
  int16_t currentM2;
  bool valid = g_roboclaw.ReadCurrents(DEVICE_ADDRESS, currentM1, currentM2);
  if (!valid) {
    // Serial.print("[TRoboClaw::getCurrents] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_current_m1 = currentM1;
    g_current_m2 = currentM2;
    g_state = LOGIC_BATTERY;
  }
}

void TRoboClaw::getEncoderM1() {
  bool valid;
  uint8_t status;
  int32_t value = g_roboclaw.ReadEncM1(DEVICE_ADDRESS, &status, &valid);
  if (!valid) {
    // Serial.print("[TRoboClaw::getEncoderM1] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_encoder_m1 = value;
    g_state = ENCODER_M2;
  }
}

void TRoboClaw::getEncoderM2() {
  bool valid;
  uint8_t status;
  int32_t value = g_roboclaw.ReadEncM2(DEVICE_ADDRESS, &status, &valid);
  if (!valid) {
    // Serial.print("[TRoboClaw::getEncoderM2] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_encoder_m2 = value;
    g_state = CURRENTS;
  }
}

uint32_t TRoboClaw::getError() { return g_roboclaw.ReadError(DEVICE_ADDRESS); }

void TRoboClaw::getLogicBattery() {
  bool valid;
  int16_t voltage;
  voltage = g_roboclaw.ReadLogicBatteryVoltage(DEVICE_ADDRESS, &valid);
  if (!valid) {
    // Serial.print("[TRoboClaw::getLogicBattery] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_logic_battery = voltage;
    g_state = MAIN_BATTERY;
  }
}

float TRoboClaw::getM1Current() { return g_current_m1 / 1000.0; }

int32_t TRoboClaw::getM1Encoder() { return g_encoder_m1; }

int32_t TRoboClaw::getM1Speed() { return g_speed_m1; }

float TRoboClaw::getM2Current() { return g_current_m2 / 1000.0; }

int32_t TRoboClaw::getM2Encoder() { return g_encoder_m2; }

int32_t TRoboClaw::getM2Speed() { return g_speed_m2; }

void TRoboClaw::getMainBattery() {
  bool valid;
  int16_t voltage;
  voltage = g_roboclaw.ReadMainBatteryVoltage(DEVICE_ADDRESS, &valid);
  if (!valid) {
    // Serial.print("[TRoboClaw::getMainBattery] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_main_battery = voltage;
    g_state = SPEED_M1;  //  Restart sequence
  }
}

void TRoboClaw::getSpeedM1() {
  bool valid;
  uint8_t status;
  uint32_t speed = g_roboclaw.ReadSpeedM1(DEVICE_ADDRESS, &status, &valid);
  if (!valid) {
    // Serial.print("[TRoboClaw::getSpeedM1] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_speed_m1 = speed;
    g_state = SPEED_M2;
  }
}

void TRoboClaw::getSpeedM2() {
  bool valid;
  uint8_t status;
  int32_t speed = g_roboclaw.ReadSpeedM2(DEVICE_ADDRESS, &status, &valid);
  if (!valid) {
    // Serial.print("[TRoboClaw::getSpeed2] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_speed_m2 = speed;
    g_state = ENCODER_M1;
  }
}

void TRoboClaw::getVersion() {
  char version[32];
  version[0] = '\0';

  if (g_roboclaw.ReadVersion(DEVICE_ADDRESS, version)) {
    if (strcmp(version, DEVICE_VERSION) != 0) {
      // Serial.print("[TRoboClaw::getVersion] version mismatch, found: '");
      // Serial.print(version);
      // Serial.println("'");
      reconnect();
    } else {
      // Serial.print("[TRoboClaw::getVersion] version match: '");
      // Serial.print(version);
      // Serial.println("'");
      g_state = SPEED_M1;
    }
  } else {
    // Serial.println("[TRoboClaw::getVersion fail");
    reconnect();
    g_state = VERSION;
  }
}

void TRoboClaw::resetEncoders() {
  g_roboclaw.SetEncM1(DEVICE_ADDRESS, 0);
  g_roboclaw.SetEncM2(DEVICE_ADDRESS, 0);
}

void TRoboClaw::setM1PID(float p, float i, float d, uint32_t qpps) {
  g_roboclaw.SetM1VelocityPID(DEVICE_ADDRESS, p, i, d, qpps);
}

void TRoboClaw::setM2PID(float p, float i, float d, uint32_t qpps) {
  g_roboclaw.SetM2VelocityPID(DEVICE_ADDRESS, p, i, d, qpps);
}

void TRoboClaw::loop() {
  static const float kEncoderCountFaultThresholdPerSecond = 1900.0;
  static const uint32_t kMaxAllowedConsecutiveEncoderFaults = uint32_t(
      1566.0 * 0.1);  // Pulses/meter * max-allowed-fault-distance-in-meters.
  static uint32_t last_m1_encoder_time_ms = millis();
  static uint32_t last_m2_encoder_time_ms = millis();
  static int32_t last_m1_encoder_value = g_encoder_m1;
  static int32_t last_m2_encoder_value = g_encoder_m2;
  static uint32_t consecutive_m1_encoder_faults = 0;
  static uint32_t consecutive_m1_loops = 0;
  static uint32_t consecutive_m2_encoder_faults = 0;
  switch (g_state) {
    case VERSION:
      getVersion();
      break;

    case SPEED_M1: {
      getSpeedM1();
    } break;

    case SPEED_M2:
      getSpeedM2();
      break;

    case ENCODER_M1: {
      getEncoderM1();
      uint32_t now_ms = millis();
      float duration = ((now_ms * 1.0) - last_m1_encoder_time_ms) / 1000.0;
      float encoder_diff_per_second =
          abs(g_encoder_m1 - last_m1_encoder_value) / duration;
      if (encoder_diff_per_second > kEncoderCountFaultThresholdPerSecond) {
        consecutive_m1_loops += 1;
        consecutive_m1_encoder_faults +=
            abs(g_encoder_m1 - last_m1_encoder_value);
        if (consecutive_m1_encoder_faults >
            kMaxAllowedConsecutiveEncoderFaults) {
          TRelay::singleton().powerOn(TRelay::MOTOR_ESTOP);
          char msg[512];
          snprintf(
              msg, sizeof(msg),
              "ERROR TRoboClaw::Loop RUNAWAY M1 duration: %-2.3f, last "
              "encoder: %ld, current encoder: %ld, diff: %ld, "
              "encoder_diff_per_second: %-2.3f "
              "consecutive_m1_encoder_faults: %ld, consecutive_m1_loops: %ld",
              duration, last_m1_encoder_value, g_encoder_m1,
              abs(g_encoder_m1 - last_m1_encoder_value),
              encoder_diff_per_second, consecutive_m1_encoder_faults,
              consecutive_m1_loops);
          TMicroRos::singleton().publishDiagnostic(msg);
          consecutive_m1_encoder_faults = 0;
          consecutive_m1_loops = 0;
        }
      } else {
        consecutive_m1_encoder_faults = 0;
        consecutive_m1_loops = 0;
        TRelay::singleton().powerOff(TRelay::MOTOR_ESTOP);  // ###
      }

      last_m1_encoder_value = g_encoder_m1;
      last_m1_encoder_time_ms = now_ms;
    } break;

    case ENCODER_M2:
      getEncoderM2();
      break;

    case CURRENTS:
      getCurrents();
      break;

    case LOGIC_BATTERY:
      getLogicBattery();
      break;

    case MAIN_BATTERY:
      getMainBattery();
      break;

    default:
      g_state = VERSION;
      break;
  }
}

void TRoboClaw::reconnect() {
  g_current_m1 = 0;
  g_current_m2 = 0;
  g_encoder_m1 = 0;
  g_encoder_m2 = 0;
  g_logic_battery = 0;
  g_main_battery = 0;
  g_roboclaw.~RoboClaw();
  g_roboclaw = RoboClaw(&Serial6, 10'000);
  g_roboclaw.begin(115'200);
  g_speed_m1 = 0;
  g_speed_m2 = 0;
}

void TRoboClaw::setup() {
  reconnect();
  getEncoderM1();
  getEncoderM2();
}

TRoboClaw::TRoboClaw()
    : TModule(TModule::kROBOCLAW),
      g_current_m1(0),
      g_current_m2(0),
      g_encoder_m1(0),
      g_encoder_m2(0),
      g_logic_battery(0),
      g_main_battery(0),
      g_speed_m1(0),
      g_speed_m2(0) {
  Serial6.begin(115'200);
}

TRoboClaw& TRoboClaw::singleton() {
  if (!g_singleton) {
    g_singleton = new TRoboClaw();
  }

  return *g_singleton;
}

const char* TRoboClaw::DEVICE_VERSION = "USB Roboclaw 2x15a v4.2.4\n";

RoboClaw TRoboClaw::g_roboclaw(&Serial6, 10'000);

TRoboClaw* TRoboClaw::g_singleton = nullptr;

TRoboClaw::TSTATE TRoboClaw::g_state = TRoboClaw::VERSION;