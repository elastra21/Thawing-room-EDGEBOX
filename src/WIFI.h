#ifndef MY_WIFI_H
#define MY_WIFI_H
// #include "Arduino.h"
#include "EEPROM.h"
#include "SPIFFS.h"
#include <Update.h>
#include "secrets.h"
#include <ESPmDNS.h>
#include <AsyncTCP.h>
// #include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#ifdef WebSerial
  // No incluir WebSerialLite.h
#else
  #include "WebSerialLite.h"
#endif

#include "ESPAsyncWebServer.h"

class WIFI {
  public:
    void localIP();
    void loopOTA();
    void setUpOTA();
    void setUpWiFi();
    void reconnect();
    bool isConnected();
    bool refreshWiFiStatus();
    bool getConnectionStatus();
    void setUpWebServer(bool brigeSerial = false);
  private:
    bool last_connection_state = false;
};
#endif
