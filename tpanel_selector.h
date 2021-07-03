#pragma once

#include "tcontrol_display.h"
#include "tmodule.h"
#include "ton_off_button.h"

class TPanelSelector : TModule {
 public:
  typedef enum { POWER_PANEL, PROXIMITY_PANEL } TPanel;

  TPanel activePanel() { return g_selectedPanel; }

  static TPanelSelector& singleton();

  void loop();

  const char* name() { return "TPanelSelector"; }

  void setup();

 private:
  TPanelSelector();

  static void powerPanelCallback(TOnOffButton& button, void* parameter);

  static void proximityPanelCallback(TOnOffButton& button, void* parameter);

  static TPanel g_selectedPanel;

  static TPanelSelector* g_singleton;

  static TControlDisplay& g_tc;
};
