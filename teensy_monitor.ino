#include <Wire.h>
#include <stdint.h>

#include "Watchdog_t4.h"
#include "talarm.h"
#include "talert.h"
#include "tmodule.h"
#include "tmotor_current.h"
#include "tpanel_selector.h"
#include "tpower_panel.h"
#include "tproximity_panel.h"
#include "trelay.h"
#include "tros_client.h"
#include "tserver.h"
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"

TAlarm& alarm = TAlarm::singleton();
TAlert& alert = TAlert::singleton();
TMotorCurrent& motorCurrent = TMotorCurrent::singleton();
TPanelSelector& panelSelector = TPanelSelector::singleton();
TRelay& relay = TRelay::singleton();
TRosClient& rosClient = TRosClient::singleton();
TServer& server = TServer::singleton();
TSonar& sonar = TSonar::singleton();
TTemperature& temperature = TTemperature::singleton();
TTimeOfFlight& timeOfFlight = TTimeOfFlight::singleton();

TModule& modules = TModule::TModule::singleton();
WDT_T4<WDT3> wdt;

void watchdogTimeout() {
  Serial.println("WATCHDOG TIMEOUT, 255 CYCLES TILL RESET...");
}

void setup() {
  Wire.begin();
  Serial.begin(38400);
  while (!Serial && (millis() <= 1000))
    ;

  WDT_timings_t config;
  config.window =
      1; /* in seconds, 32ms to 522.232s, must be smaller than timeout */
  config.timeout = 160000; /* in seconds, 32ms to 522.232s */
  config.callback = watchdogTimeout;
  wdt.begin(config);
  modules.setup();
}

void loop() {
  modules.loop();
  wdt.feed();
}
