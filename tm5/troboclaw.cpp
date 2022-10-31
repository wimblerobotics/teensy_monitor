#include "troboclaw.h"

#include <string.h>

#include <limits>

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
  char msg[512];
  snprintf(msg, sizeof(msg),
           "accel_qpps: %ld, m1_qpps: %ld, m1_max_dist: %ld, m2_qpps: %ld, "
           "m2_max_dist: %ld",
           accel_quad_pulses_per_second, m1_quad_pulses_per_second,
           m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
  TMicroRos::singleton().publishDiagnostic(msg);
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
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getCurrents] fail");
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
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getEncoderM1] fail");
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
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getEncoderM2] fail");
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
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getLogicBattery] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_logic_battery = voltage;
    g_state = MAIN_BATTERY;
  }
}

float TRoboClaw::getM1Current() { return g_current_m1 / 100.0; }

int32_t TRoboClaw::getM1Encoder() { return g_encoder_m1; }

int32_t TRoboClaw::getM1Speed() { return g_speed_m1; }

float TRoboClaw::getM2Current() { return g_current_m2 / 100.0; }

int32_t TRoboClaw::getM2Encoder() { return g_encoder_m2; }

int32_t TRoboClaw::getM2Speed() { return g_speed_m2; }

void TRoboClaw::getMainBattery() {
  bool valid;
  int16_t voltage;
  voltage = g_roboclaw.ReadMainBatteryVoltage(DEVICE_ADDRESS, &valid);
  if (!valid) {
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getMainBattery] fail");
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
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getSpeedM1] fail");
    reconnect();
    g_speed_m1 = std::numeric_limits<uint32_t>::min();
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
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getSpeed2] fail");
    reconnect();
    g_speed_m2 = std::numeric_limits<uint32_t>::min();
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
    char msg[512];
    if (strcmp(version, DEVICE_VERSION) != 0) {
      snprintf(msg, sizeof(msg),
               "ERROR [TRoboClaw::getVersion] version mismatch, found: '%s'",
               version);
      TMicroRos::singleton().publishDiagnostic(msg);
      reconnect();
    } else {
      snprintf(msg, sizeof(msg),
               "info [TRoboClaw::getVersion] version match, found: '%s'",
               version);
      TMicroRos::singleton().publishDiagnostic(msg);
      g_state = SPEED_M1;
    }
  } else {
    TMicroRos::singleton().publishDiagnostic(
        "ERROR [TRoboClaw::getVersion fail");
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

void TRoboClaw::checkForRunaway(TRoboClaw::WhichMotor whichMotor) {
  static const float kEncoderCountFaultThresholdPerSecond = 1900.0;
  static const uint32_t kMaxAllowedConsecutiveEncoderFaults = uint32_t(
      1566.0 * 0.1);  // Pulses/meter * <max-allowed-fault-distance-in-meters>.
  static uint32_t last_checked_m1_encoder_time_ms = millis();
  static uint32_t last_checked_m2_encoder_time_ms = millis();

  static int32_t last_checked_m1_encoder_value = g_encoder_m1;
  static int32_t last_checked_m2_encoder_value = g_encoder_m2;

  static uint32_t accumulated_m1_encoder_diffs = 0;
  static uint32_t accumulated_m2_encoder_diffs = 0;

  static bool is_setup = false;

  float duration_since_last_runaway_check_for_encoder = 0;
  float encoder_diff_per_second = 0;
  bool runaway_fault = false;

  const char* motor_name;

  if (!is_setup) {
    last_checked_m1_encoder_value = g_encoder_m1;
    last_checked_m2_encoder_value = g_encoder_m2;
    is_setup = true;
  }

  uint32_t now_ms = millis();
  switch (whichMotor) {
    case kLEFT_MOTOR:
      duration_since_last_runaway_check_for_encoder =
          ((now_ms * 1.0) - last_checked_m1_encoder_time_ms) / 1000.0;
      encoder_diff_per_second =
          abs(g_encoder_m1 - last_checked_m1_encoder_value) /
          duration_since_last_runaway_check_for_encoder;
      motor_name = "M1";
      break;
    case kRIGHT_MOTOR:
      duration_since_last_runaway_check_for_encoder =
          ((now_ms * 1.0) - last_checked_m2_encoder_time_ms) / 1000.0;
      encoder_diff_per_second =
          abs(g_encoder_m2 - last_checked_m2_encoder_value) /
          duration_since_last_runaway_check_for_encoder;
      motor_name = "M2";
      break;
  }

  if (encoder_diff_per_second > kEncoderCountFaultThresholdPerSecond) {
    // The encodeers are spinning too fast.
    // A fault is triggered only if the robot travels at least a certain
    // distance at this high speed.
    if (whichMotor == kLEFT_MOTOR) {
      accumulated_m1_encoder_diffs +=
          abs(g_encoder_m1 - last_checked_m1_encoder_value);
      runaway_fault =
          accumulated_m1_encoder_diffs > kMaxAllowedConsecutiveEncoderFaults;
    } else {
      accumulated_m2_encoder_diffs +=
          abs(g_encoder_m2 - last_checked_m2_encoder_value);
      runaway_fault =
          accumulated_m2_encoder_diffs > kMaxAllowedConsecutiveEncoderFaults;
    }

    if (runaway_fault) {
      TRelay::singleton().powerOn(TRelay::MOTOR_ESTOP);  // E-stop the motors.
      char msg[512];
      snprintf(msg, sizeof(msg),
               "ERROR TRoboClaw::Loop RUNAWAY for motor: %s, "
               "duration_since_last_runaway_check_for_encoder: %-2.3f",
               motor_name, duration_since_last_runaway_check_for_encoder);
      TMicroRos::singleton().publishDiagnostic(msg);
      if (whichMotor == kLEFT_MOTOR) {
        accumulated_m1_encoder_diffs = 0;
      } else {
        accumulated_m2_encoder_diffs = 0;
      }
    }
  } else {
    // The motors are not spinning too fast now.
    if (whichMotor == kLEFT_MOTOR) {
      accumulated_m1_encoder_diffs = 0;
    } else {
      accumulated_m2_encoder_diffs = 0;
    }
    // TRelay::singleton().powerOff(TRelay::MOTOR_ESTOP);  // ###
  }

  if (whichMotor == kLEFT_MOTOR) {
    last_checked_m1_encoder_value = g_encoder_m1;
    last_checked_m1_encoder_time_ms = now_ms;
  } else {
    last_checked_m2_encoder_value = g_encoder_m2;
    last_checked_m2_encoder_time_ms = now_ms;
  }
}

void TRoboClaw::loop() {
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

    case ENCODER_M1:
      getEncoderM1();
      checkForRunaway(kLEFT_MOTOR);
      break;

    case ENCODER_M2:
      getEncoderM2();
      checkForRunaway(kRIGHT_MOTOR);
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