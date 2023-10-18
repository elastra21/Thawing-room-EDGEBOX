// #include "WiFiType.h"
#include "WIFI.h"

AsyncWebServer server(80);

static void handle_update_progress_cb(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index){
    int cmd = (filename.indexOf("spiffs") > -1) ? U_SPIFFS : U_FLASH;
    // Update.runAsync(true);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }

  if (final) {
    if (!Update.end(true)){
      Update.printError(Serial);
    }
  }
}

/* Message callback of WebSerial */
static void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

void WIFI::init(const char* ssid, const char* password, const char* hostname){
  strncpy(this->ssid, ssid, sizeof(this->ssid) - 1);
  this->ssid[sizeof(this->ssid) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(this->password, password, sizeof(this->password) - 1);
  this->password[sizeof(this->password) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(this->hostname, hostname, sizeof(this->hostname) - 1);
  this->hostname[sizeof(this->hostname) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'
}

void WIFI::setUpWebServer(bool brigeSerial){
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }


  /*use mdns for host name resolution*/
  if (!MDNS.begin(hostname)){ // http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1){
      delay(1000);
    }
  }
  Serial.println("mDNS responder started Pinche Hugo");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
    // server.send(200, "text/html", loginIndex); 
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/serverIndex", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/server-index.html", "text/html");
  });
  /*handling uploading firmware file */
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Resetting...");
    ESP.restart(); 
  });

  server.on("/update", HTTP_POST, []( AsyncWebServerRequest *request) {
    request->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); 
  }, handle_update_progress_cb);
  
  if (brigeSerial) {
    #ifdef WebSerial_h // Verifica si WebSerialLite.h está incluido 
      WebSerial.begin(&server);
      WebSerial.onMessage(recvMsg);
    #endif // WebSerialLite_h
  }
  server.begin();
}

void WIFI::connectToWiFi(){
  WiFi.begin(ssid, password);
  uint32_t notConnectedCounter = 0;
  EEPROM.begin(32);
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println("Wifi connecting...");
      
    notConnectedCounter++;
    if(notConnectedCounter > 7) { // Reset board if not connected after 5s
      Serial.println("Resetting due to Wifi not connecting...");
      const uint8_t num_of_tries = EEPROM.readInt(1);
      if (num_of_tries == 3) break;          
      else {
        EEPROM.writeInt(1, num_of_tries + 1);
        EEPROM.commit();
        EEPROM.end();
        ESP.restart();          
      }
    }
  }

  EEPROM.writeInt(1, 0);
  EEPROM.commit();
  EEPROM.end();

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void WIFI::setUpOTA(){
  if(isConnected()){
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.onStart([]() {
    String type;
    type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem";
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    }).onEnd([]() {
      Serial.println("\nEnd");
    }).onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    }).onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
  }
}

void WIFI::loopOTA(){
  ArduinoOTA.handle();
}

bool WIFI::refreshWiFiStatus(){
  const bool connection = isConnected();
  if (connection != last_connection_state){
    last_connection_state = connection;
    return true;
  }
  return false;
}

bool WIFI::getConnectionStatus(){
  return last_connection_state;
}

bool WIFI::isConnected(){
  return WiFi.status() == WL_CONNECTED;
}

void WIFI::reconnect(){
  WiFi.begin(ssid, password);
  uint8_t timeout = 0;
  vTaskDelay( 2000 );
  while ( WiFi.status() != WL_CONNECTED ){
    vTaskDelay( 2000 );
    log_i(" waiting on wifi connection" );
    timeout++;
    if (timeout == 2) return;
  }
}



