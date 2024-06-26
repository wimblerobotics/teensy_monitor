#include <Wire.h>
#include <micro_ros_arduino.h>
#include <stdint.h>

#include "tconfiguration.h"
// #include "Watchdog_t4.h"
#include "tmicro_ros.h"
#include "tmodule.h"
#include "trelay.h"
#include "troboclaw.h"
#if USE_TSD
#include "tsd.h"
#endif
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"

// Initialize all TModule instances in any required order.
TMicroRos& micro_ros = TMicroRos::singleton();
// TProximityPanel& proximityPanel = TProximityPanel::singleton();
TRelay& relay = TRelay::singleton();
TRoboClaw& roboclaw = TRoboClaw::singleton();
#if USE_TSD
TSd& sd = TSd::singleton();
#endif
TSonar& sonar = TSonar::singleton();
TTemperature& temperature = TTemperature::singleton();

// if (TM5::kDoTOF) {
//   TTimeOfFlight& time_of_flight = TTimeOfFlight::singleton();
// }

// WDT_T4<WDT3> wdt;

// Method to handle watchdog timeout.
void watchdogTimeout() {
  // Serial.println("WATCHDOG TIMEOUT, 255 CYCLES TILL RESET...");
}

void setup() {
  Wire.begin();
  pinMode(13, OUTPUT);
  
  // Serial.begin(38400);
  // while (!Serial && (millis() <= 1000))
  //   ;
  //   Serial.println("Start");

  //  WDT_timings_t config;
  //  config.window = 1;       // Minimum time (ms) betwee//n watchdog feed()
  //  calls. config.timeout = 20000;  // Maximum time (ms) between watchdog
  //  feed() calls. config.callback = watchdogTimeout;

  TModule::DoSetup();

  //  wdt.begin(config);
}

void loop() {
  TModule::DoLoop();
  //  wdt.feed();
}
