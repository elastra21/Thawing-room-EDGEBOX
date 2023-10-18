#ifndef MY_WIFI_H
#define MY_WIFI_H
#include "EEPROM.h"
#include "SPIFFS.h"
#include <Update.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
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
    void init(const char* ssid, const char* password, const char* hostname);
    void loopOTA();
    void setUpOTA();
    void reconnect();
    bool isConnected();
    void connectToWiFi();
    bool refreshWiFiStatus();
    bool getConnectionStatus();
    void setUpWebServer(bool brigeSerial = false);
  private:
    char ssid[32];  
    char password[64];
    char hostname[32];  
    bool last_connection_state = false;
};
#endif
