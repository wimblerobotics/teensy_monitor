#include "talert.h"

#include <Arduino.h>
#include <stdint.h>

#include "talarm.h"
#include "ton_off_button.h"
#include "trelay.h"


void TAlert::loop() {
  bool turnMotorsOff = false;
  for (uint8_t i = 0; i < (uint8_t) NUMER_ALERT_SOURCES; i++) {
    switch ((TAlertSource) i) {
    case CURRENT_LEFT_MOTOR:
			break;

    case CURRENT_RIGHT_MOTOR:
			break;

    case SONAR_BACK:
			break;

    case SONAR_FRONT:
      if (g_alertsTriggered[i]) {
        //#####Serial.println("[TAlert::loop] case SONAR_FRONT alert triggered");
        turnMotorsOff = true;
      }

			break;

    case SONAR_LEFT:
			break;

    case SONAR_RIGHT:
			break;

    case TEMP_LEFT_MOTOR:
			break;

    case TEMP_RIGHT_MOTOR:
			break;

    case TOF_LOWER_LEFT_BACKWARD:
			break;

    case TOF_LOWER_LEFT_SIDEWAY:
			break;

    case TOF_LOWER_RIGHT_BACKWARD:
			break;

    case TOF_LOWER_RIGHT_SIDEWAY:
			break;

    case TOF_UPPER_LEFT_FORWARD:
      // if (g_alertsTriggered[i]) Serial.println("TOF_UPPER_LEFT_FORWARD");
			break;

    case TOF_UPPER_LEFT_SIDEWAY:
			break;

    case TOF_UPPER_RIGHT_FORWARD:
			break;

    case TOF_UPPER_RIGHT_SIDEWAY:
			break;

    default:
      break;
    } // switch
  } // for

  if (turnMotorsOff) {
    //#####Serial.println("[TAlert::loop] need to turn motors off");
    TRelay::singleton().unset(TRelay::MOTOR_POWER);
    TAlarm::singleton().set(TAlarm::MOTOR_ALARM);
    TOnOffButton::setState(1, TOnOffButton::OFF);
  }
}


void TAlert::set(TAlertSource alert) {
  g_alertsTriggered[(uint8_t) alert] = true;
}


void TAlert::setup() {

}


TAlert& TAlert::singleton() {
  if (!g_singleton) {
    g_singleton = new TAlert();
  }

  return *g_singleton;
}


void TAlert::unset(TAlertSource alert) {
  g_alertsTriggered[(uint8_t) alert] = false;
}


bool TAlert::g_alertsTriggered[NUMER_ALERT_SOURCES];

TAlert* TAlert::g_singleton = nullptr;
