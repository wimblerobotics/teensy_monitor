#include "tserver.h"


#include <NativeEthernet.h>
#include <stdio.h>

#include "tmotor_current.h"
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"


TServer::TServer() {

}


std::string TServer::sensorString() {
  int motorCurrentValues[2];
  int sonarValues[TSonar::NUMBER_SENSORS];
  int temperatureValues[TTemperature::NUMBER_SENSORS];
  int timeOfFlightValues[TTimeOfFlight::NUMBER_SENSORS];

  for (uint8_t i = 0; i < 2; i++) {
    motorCurrentValues[i] = TMotorCurrent::singleton().getValueMa(i);
  }

  for (uint8_t i = 0; i < TSonar::NUMBER_SENSORS; i++) {
    sonarValues[i] = TSonar::singleton().getValueMm(i);
  }

  for (uint8_t i = 0; i < TTemperature::NUMBER_SENSORS; i++) {
    temperatureValues[i] = TTemperature::singleton().getValueTenthsC(i);
  }

  for (uint8_t i = 0; i < TTimeOfFlight::NUMBER_SENSORS; i++) {
    timeOfFlightValues[i] = TTimeOfFlight::singleton().getValueMm(i);
  }

  char result[512];
  sprintf(
    result,
    "{\n"
    "  \"motor_currents_ma\": [ %d, %d],\n"
    "  \"sonar_mm\": [%d, %d, %d, %d],\n"
    "  \"temperature_tenthsC\": [%d, %d],\n"
    "  \"time_of_flight_mm\": [%d, %d, %d, %d, %d, %d, %d, %d]\n"
    "}\n",
    motorCurrentValues[0],
    motorCurrentValues[1],
    sonarValues[0],
    sonarValues[1],
    sonarValues[2],
    sonarValues[3],
    temperatureValues[0],
    temperatureValues[1],
    timeOfFlightValues[0],
    timeOfFlightValues[1],
    timeOfFlightValues[2],
    timeOfFlightValues[3],
    timeOfFlightValues[4],
    timeOfFlightValues[5],
    timeOfFlightValues[6],
    timeOfFlightValues[7]
    );
  
  // Serial.print("[TServer::handleRequest] result length: ");Serial.println(strlen(result));
  std::string str(result);
  return str;
}


void TServer::loop() {
  if (!g_isInitialized) {
    initializeWhenLinkIsUp();
    if (!g_isInitialized) {
      return;
    }
  }

  static char str[2048];
  static uint16_t strIndex = 0;

  switch (g_serverStatus) {
    case AWAIT_CLIENT:
      g_client = g_server.available();
      if (g_client) {
        g_serverStatus = READ_REQUEST;
      }

      break;

    case READ_REQUEST:
      if (!g_client.connected() || !g_client.available()) {
        // Connection dropped.
        g_client.stop();
        Serial.println("[TServer::loop] client disconnected");
        g_serverStatus = AWAIT_CLIENT;
      } else {
        char c = g_client.read();
        if (strIndex < (sizeof(str) - 1)) {
          str[strIndex++] = c;
        }

        str[strIndex] = 0;
        if (c == '\n') {
          // Serial.print("[TServer::loop] request: '");
          // Serial.print(str);
          // Serial.println("'");

          std::string result = sensorString();
          char content[512];
          sprintf(content, g_header, strlen(result.c_str()), result.c_str());
          g_client.println(content);
          g_serverStatus = AWAIT_CLIENT;
        }
      }

      break;
    
    default:
      break;
  }
}


void TServer::initializeWhenLinkIsUp() {
  if (Ethernet.linkStatus() == LinkON) {
    Ethernet.begin((uint8_t*) &MAC_ADDRESS, IPAddress(192, 168, 2, 120));
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      g_serverStatus = NO_DEVICE;
      Serial.println("Ethernet device is not found");
    } else {
      if (Ethernet.linkStatus() == LinkOFF) {
        g_serverStatus = NO_LINK;
        Serial.println("Ethernet cable not connected");
      } else {
        g_server.begin();
        Serial.print("TServer IP address is: ");
        Serial.println(Ethernet.localIP());
        g_serverStatus = AWAIT_CLIENT;
        g_isInitialized = true;
      }
    }
  } else {
    Ethernet.begin((uint8_t*) &MAC_ADDRESS, IPAddress(192, 168, 2, 120));
  }
}


void TServer::setup() {
  initializeWhenLinkIsUp();
}


TServer& TServer::singleton() {
  if (!g_singleton) {
    g_singleton = new TServer();
  }
  
  return *g_singleton;
}


EthernetClient TServer::g_client;

bool TServer::g_isInitialized = false;

EthernetServer TServer::g_server(80);

TServer::TServerStatus TServer::g_serverStatus = NO_DEVICE;

TServer* TServer::g_singleton = nullptr;

const uint8_t TServer::MAC_ADDRESS[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

const char* TServer::g_header = "HTTP/1.1 200 OK\n"
  // "Date: Mon, 27 Jul 2009 12:28:53 GMT\n"
  // "Server: Apache/2.2.14 (Win32)\n"
  // "Last-Modified: Wed, 22 Jul 2009 19:15:56 GMT\n"
  "Content-Type: application/json\n"
  // "Content-Type: text/html\n"
 "Content-Length: %d\n"
  // "Connection: closed\n"
  // "Refresh: 5\n"
  "\r\n"
  "%s";