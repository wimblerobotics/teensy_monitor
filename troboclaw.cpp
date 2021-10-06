#include "troboclaw.h"

#include <string.h>

#include "Arduino.h"
#include "RoboClaw.h"

#define HWSERIAL Serial6

float TRoboClaw::getBatteryLogic() {
  return g_logic_battery / 10.0;
}

float TRoboClaw::getBatteryMain() {
  return g_main_battery / 10.0;
}

void TRoboClaw::getCurrents() {
  int16_t currentM1;
  int16_t currentM2;
  bool valid = g_roboclaw.ReadCurrents(DEVICE_ADDRESS, currentM1, currentM2);
  if (!valid) {
    Serial.print("[TRoboClaw::getCurrents] fail");
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
    Serial.print("[TRoboClaw::getEncoderM1] fail");
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
    Serial.print("[TRoboClaw::getEncoderM2] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_encoder_m2 = value;
    g_state = CURRENTS;
  }
}

void TRoboClaw::getLogicBattery() {
  bool valid;
  int16_t voltage;
  voltage = g_roboclaw.ReadLogicBatteryVoltage(DEVICE_ADDRESS, &valid);
  if (!valid) {
    Serial.print("[TRoboClaw::getLogicBattery] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_logic_battery = voltage;
    g_state = MAIN_BATTERY;
  }
}

float TRoboClaw::getM1Current() {
  return g_current_m1 / 1000.0;
}

int32_t TRoboClaw::getM1Encoder() {
  return g_encoder_m1;
}

int32_t TRoboClaw::getM1Speed() {
  return g_speed_m1;
}

float TRoboClaw::getM2Current() {
  return g_current_m2 / 1000.0;
}

int32_t TRoboClaw::getM2Encoder() {
  return g_encoder_m2;
}

int32_t TRoboClaw::getM2Speed() {
  return g_speed_m2;
}

void TRoboClaw::getMainBattery() {
  bool valid;
  int16_t voltage;
  voltage = g_roboclaw.ReadMainBatteryVoltage(DEVICE_ADDRESS, &valid);
  if (!valid) {
    Serial.print("[TRoboClaw::getMainBattery] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_main_battery = voltage;
    g_state = SPEED_M1; //  Restart sequence
  }
}

void TRoboClaw::getSpeedM1() {
  bool valid;
  uint8_t status;
  uint32_t speed = g_roboclaw.ReadSpeedM1(DEVICE_ADDRESS, &status, &valid);
  if (!valid) {
    Serial.print("[TRoboClaw::getSpeedM1] fail");
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
    Serial.print("[TRoboClaw::getSpeed2] fail");
    reconnect();
    g_state = VERSION;
  } else {
    g_speed_m2 = speed;
    g_state = ENCODER_M1;
  }
}

void TRoboClaw::getVersion() {
  char version[32];

  if (g_roboclaw.ReadVersion(DEVICE_ADDRESS, version)) {
    if (strcmp(version, DEVICE_VERSION) != 0) {
      Serial.print("[TRoboClaw::getVersion] version mismatch, found: '");
      Serial.print(version);
      Serial.println("'");
      reconnect();
    } else {
      Serial.print("[TRoboClaw::getVersion] version match: '");
      Serial.print(version);
      Serial.println("'");
      g_state = SPEED_M1;
    }
  } else {
    Serial.println("[TRoboClaw::getVersion fail");
    reconnect();
    g_state = VERSION;
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
      break;

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
  g_speed_m1 = 0;
  g_speed_m2 = 0;
}

void TRoboClaw::setup() { g_roboclaw.begin(115'200); }

TRoboClaw::TRoboClaw()
    : TModule(),
      g_current_m1(0),
      g_current_m2(0),
      g_encoder_m1(0),
      g_encoder_m2(0),
      g_logic_battery(0),
      g_main_battery(0),
      g_speed_m1(0),
      g_speed_m2(0) {}

TRoboClaw& TRoboClaw::singleton() {
  if (!g_singleton) {
    g_singleton = new TRoboClaw();
  }

  return *g_singleton;
}

const char* TRoboClaw::DEVICE_VERSION = "USB Roboclaw 2x15a v4.1.34\n";

RoboClaw TRoboClaw::g_roboclaw(&Serial6, 10'000);

TRoboClaw* TRoboClaw::g_singleton = nullptr;

TRoboClaw::TSTATE TRoboClaw::g_state = TRoboClaw::VERSION;