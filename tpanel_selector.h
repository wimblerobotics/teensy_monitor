#pragma once

#include "tcontrol_display.h"
#include "ton_off_button.h"

class TPanelSelector {
public:

    typedef enum {
        POWER_PANEL,
        PROXIMITY_PANEL
    } TPanel;

    TPanel activePanel() { return g_selectedPanel; }

    static TPanelSelector& singleton();

    void loop();

    void setupPanel();

private:

    TPanelSelector();

    static void powerPanelCallback(TOnOffButton& button, void* parameter);

    static void proximityPanelCallback(TOnOffButton& button, void* parameter);

    static TPanel g_selectedPanel;
    
    static TPanelSelector* g_singleton;
   
    static TControlDisplay& g_tc;
};
