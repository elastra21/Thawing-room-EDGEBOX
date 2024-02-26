#ifndef MY_WIFI_H
#define MY_WIFI_H
#include "EEPROM.h"
#include <Update.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include "logger.h"
#include "resources/WebFiles.h"
// AP mode, This is not well implemented yet!!!!!!!
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
///////////////////////////////////////////////////

#define SSID_SIZE 32
#define PASSWORD_SIZE 64
#define HOSTNAME_SIZE 32
#define IP_ADDRESS_SIZE 16
#define MQTT_ID_SIZE 32
#define USERNAME_SIZE 32
#define MQTT_PASSWORD_SIZE 32
#define PREFIX_SIZE 32


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
    String getIP();
    void setUpOTA();
    void reconnect();
    bool isConnected();
    void startAPMode();
    void connectToWiFi();
    bool refreshWiFiStatus();
    bool getConnectionStatus();
    void setUpWebServer(bool brigeSerial = false);
  private:
    char ssid[SSID_SIZE];  
    char password[PASSWORD_SIZE];
    char hostname[HOSTNAME_SIZE]; 
    bool last_connection_state = false;
};
#endif
