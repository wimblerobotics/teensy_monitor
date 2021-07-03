#pragma once

#include <stdint.h>
#include <string>
#include <NativeEthernet.h>

#include "tmodule.h"

class TServer : TModule {
public:

  static TServer& singleton();

  void loop();

  const char* name() { return "TServer"; }

  void setup();

private:
  typedef enum {
    NO_DEVICE,
    NO_LINK,
    AWAIT_CLIENT,
    READ_REQUEST
  } TServerStatus;

  TServer();

  void initializeWhenLinkIsUp();

  static std::string sensorString();

  static EthernetClient g_client;

  static bool g_isInitialized;

  static EthernetServer g_server;

  static TServerStatus g_serverStatus;

  static TServer* g_singleton;

  static const uint8_t MAC_ADDRESS[6];

  static const char* g_header;
};