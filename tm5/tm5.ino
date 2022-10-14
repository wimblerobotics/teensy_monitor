#include <Wire.h>
#include <micro_ros_arduino.h>
#include <stdint.h>
#include <yaml.h>

//#include "Watchdog_t4.h"
#include "talarm.h"
#include "talert.h"
#include "tmicro_ros.h"
#include "tmodule.h"
#include "tmotor_current.h"
#include "tpanel_selector.h"
#include "trelay.h"
#include "troboclaw.h"
//######include "tsd.h"
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"

// Initialize all TModule instances in any required order.
TAlarm& alarm = TAlarm::singleton();
TAlert& alert = TAlert::singleton();
TMicroRos& microRos = TMicroRos::singleton();
TMotorCurrent& motorCurrent = TMotorCurrent::singleton();
TPanelSelector& panelSelector = TPanelSelector::singleton();
TRelay& relay = TRelay::singleton();
TRoboClaw& roboclaw = TRoboClaw::singleton();
//#####TSd& sd = TSd::singleton();
TSonar& sonar = TSonar::singleton();
TTemperature& temperature = TTemperature::singleton();
TTimeOfFlight& timeOfFlight = TTimeOfFlight::singleton();

// WDT_T4<WDT3> wdt;

// Method to handle watchdog timeout.
void watchdogTimeout() {
  // Serial.println("WATCHDOG TIMEOUT, 255 CYCLES TILL RESET...");
}

void setup() {
  Wire.begin();
  // Serial.begin(38400);
  // while (!Serial && (millis() <= 1000))
  //   ;

  //  WDT_timings_t config;
  //  config.window = 1;       // Minimum time (ms) betwee//n watchdog feed()
  //  calls. config.timeout = 20000;  // Maximum time (ms) between watchdog
  //  feed() calls. config.callback = watchdogTimeout;
  TModule::doSetup();

  //  wdt.begin(config);
}

void loop() {
  TModule::doLoop();
  //  wdt.feed();
}
