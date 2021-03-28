#undef DO_SERVER

#include <stdint.h>
#include <Wire.h>

#include "talarm.h"
#include "talert.h"
#include "tmotor_current.h"
#include "tpanel_selector.h"
#include "tpower_panel.h"
#include "tproximity_panel.h"
#include "trelay.h"

#ifdef DO_SERVER
#include "tserver.h"
#endif

#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"



TAlarm& alarm = TAlarm::singleton();
TAlert& alert = TAlert::singleton();
TMotorCurrent& motorCurrent = TMotorCurrent::singleton();
TPanelSelector& panelSelector = TPanelSelector::singleton();
TRelay& relay = TRelay::singleton();

#ifdef DO_SERVER
TServer& server = TServer::singleton();
#endif

TSonar& sonar = TSonar::singleton();
TTemperature& temperature = TTemperature::singleton();
TTimeOfFlight& timeOfFlight = TTimeOfFlight::singleton();

void setup() {
  Wire.begin();
  Serial.begin(38400);
  while (!Serial && (millis() <= 1000));

  alarm.setup();
  alert.setup();
  panelSelector.setupPanel();
  motorCurrent.setup();
  relay.setup();
  sonar.setup();
  temperature.setup();
  timeOfFlight.setup();
#ifdef DO_SERVER
  server.setup();
#endif

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


void loop() {
  alarm.loop();
  alert.loop();
  panelSelector.loop();
  motorCurrent.loop();
  relay.loop();
  temperature.loop();
  timeOfFlight.loop();
  sonar.loop();
#ifdef DO_SERVER
  server.loop();
#endif
}
