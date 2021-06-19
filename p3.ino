#include <stdint.h>
#include <Wire.h>

#include "talarm.h"
#include "talert.h"
#include "tmotor_current.h"
#include "tpanel_selector.h"
#include "tpower_panel.h"
#include "tproximity_panel.h"
#include "trelay.h"
#include "tserver.h"
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"


TAlarm& alarm = TAlarm::singleton();
TAlert& alert = TAlert::singleton();
TMotorCurrent& motorCurrent = TMotorCurrent::singleton();
TPanelSelector& panelSelector = TPanelSelector::singleton();
TRelay& relay = TRelay::singleton();
TServer& server = TServer::singleton();
TSonar& sonar = TSonar::singleton();
TTemperature& temperature = TTemperature::singleton();
TTimeOfFlight& timeOfFlight = TTimeOfFlight::singleton();

void setup() {
  Wire.begin();
  // Serial.begin(38400);
  // while (!Serial && (millis() <= 1000));

  alarm.setup();
  alert.setup();
  panelSelector.setupPanel();
  motorCurrent.setup();
  relay.setup();
  sonar.setup();
  temperature.setup();
  timeOfFlight.setup();
  server.setup();

  // static const char* DURATION_NAMES[] = 
  //   {"Panel", "Tof  ", "Sonar", "Ether", "OnOf ", "Power", "Proxi", "Alert", "FillR"};
  // for (uint8_t f = 0; f < 9; f++) {
  //   uint32_t start = micros();
  //   for (uint16_t i = 0; i < 1000; i++) {
  //     switch (f) {
  //       case 0: panelSelector.loop(); break;
  //       case 1: timeOfFlight.loop(); break;
  //       case 2: sonar.loop(); break;
  //       case 3: server.loop(); break;
  //       case 4: TOnOffButton::update(); break;
  //       case 5: TPowerPanel::singleton().loop(); break;
  //       case 6: TProximityPanel::singleton().loop(); break;
  //       case 7: TProximityPanel::singleton().hasAlert(); break;
  //       case 8: TControlDisplay::singleton().fillRect(270, 190, 50, 50, ILI9341_RED);
  //     }
  //   }

  //   float duration = ((micros() * 1.0) - start) / 1000.0;
  //   Serial.print(DURATION_NAMES[f]);
  //   Serial.print(": ");
  //   Serial.println(duration);
  // }
}

void timeAlarm() {
  alarm.loop();
}
void timeAlert() {
  alert.loop();
}
void timeMotor() {
  motorCurrent.loop();
}
void timePanel() {
  panelSelector.loop();
}
void timeRelay() {
  relay.loop();
}
void timeServer() {
  server.loop();
}
void timeSonar() {
  sonar.loop();
}
void timeTemperature() {
  temperature.loop();
}
void timeTimeOfFlight() {
  timeOfFlight.loop();
}
void timeFunction(void (*fn)(), const char* name) {
  uint32_t start = micros();
  fn();
  float duration = ((micros() * 1.0) - start) / 1000.0;
  Serial.print("fn: ");Serial.print(name);
  Serial.print(", duration (ms): ");Serial.println(duration);
}

// Timings of slow functions:
// TOF: 1223
// PANEL 964
// MOTOR 101
//
/** After power restart
fn: TOF, duration (ms): 1.44
fn: SONAR, duration (ms): 0.00
fn: SERVER, duration (ms): 0.00
fn: ALARM, duration (ms): 0.00
fn: ALERT, duration (ms): 0.00
fn: PANEL, duration (ms): 0.00
fn: MOTOR, duration (ms): 0.98
fn: RELAY, duration (ms): 0.00
fn: TEMP, duration (ms): 0.03

 */

 // Loop duration 20170615: 1.06ms typ, 2.45 (once every 9 loops), 58.13 seldom (once every 33-40 loops)

void loop() {
//    uint32_t start = micros();

  alarm.loop(); // Fast. .000679
//  timeFunction(timeAlarm, "ALARM");
  alert.loop(); // Fast. .001225
//  timeFunction(timeAlert, "ALERT");
  panelSelector.loop(); // Fast. .00805
//  timeFunction(timePanel, "PANEL");
  motorCurrent.loop(); // Fast .001316
//  timeFunction(timeMotor, "MOTOR");
  relay.loop();  // Fast .000961
//  timeFunction(timeRelay, "RELAY");
  temperature.loop(); // Fast Accumulated for all fns up to here, 95 ms/loop
//  timeFunction(timeTemperature, "TEMP");
  timeOfFlight.loop(); // VERY SLOW, about 2.2 sec/loop
//  timeFunction(timeTimeOfFlight, "TOF");
  sonar.loop(); // Fast.
//  timeFunction(timeSonar, "SONAR");
  server.loop();
//  timeFunction(timeServer, "SERVER");
//  float duration = ((micros() * 1.0) - start) / 1000.0;
//  Serial.print("Loop duration (ms): ");Serial.println(duration);
}
