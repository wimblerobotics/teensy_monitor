#pragma once

#include <NativeEthernet.h>

#include "tmodule.h"

class TRosClient : TModule {
 public:
 
  static TRosClient &singleton();

  void loop();

  const char *name() { return "TRosClient"; }

  void setup();

  typedef enum {
    NONE,
    AWAIT_CLIENT,
    AWAIT_RESPONSE,
    GATHER_RESPONSE,
  } TState;

 private:
  TRosClient();

  void makeHttpRequest();

  static EthernetClient g_client;

  static bool g_isInitialized;

  static TRosClient *g_singleton;

  static TState g_state;

  static const uint8_t MAC_ADDRESS[6];
};