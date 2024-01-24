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

//This is not well implemented yet!!!!!!!

static void saveCredentials(const String& new_ssid, const String& new_pass) {
  // Read the existing configuration
  File configFile = SPIFFS.open("/config.txt", FILE_READ);
  if (!configFile) {
    Serial.println("Failed to open config file for reading");
    return;
  }

  // Allocate a temporary buffer for the content
  String content = configFile.readString();
  configFile.close();

  // Parse the JSON object from the file
  DynamicJsonDocument doc(1024);
  auto error = deserializeJson(doc, content);
  if (error) {
    Serial.println("Failed to parse config file");
    return;
  }

  // Update the values
  doc["SSID"] = new_ssid;
  doc["WIFI_PASSWORD"] = new_pass;

  // Open file for writing
  configFile = SPIFFS.open("/config.txt", FILE_WRITE);
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }

  // Serialize JSON to file
  if (serializeJson(doc, configFile) == 0) {
    Serial.println("Failed to write to config file");
  }

  configFile.close();
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
  /*use mdns for host name resolution*/
  if (!MDNS.begin(hostname)){ // http://esp32.local
    logger.println("Error setting up MDNS responder!");
    while (1){
      delay(1000);
    }
  }
  logger.println("mDNS responder started Pinche Hugo");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", INDEX_HTML);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/css", STYLE_CSS);
  });

  server.on("/serverIndex", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", SERVER_INDEX_HTML);
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
  
  // if (brigeSerial) {
    #ifdef WebSerial_h // Verifica si WebSerialLite.h está incluido 
      WebSerial.begin(&server);
      WebSerial.onMessage(recvMsg);
    #endif // WebSerialLite_h
  // }
  server.begin();
}

// void WIFI::startMDNS(){
//   if (!MDNS.begin("tap-o-meter")) {
//         Serial.println("Error setting up MDNS responder!");
//         while(1) vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

String WIFI::getIP(){
  String ip =  MDNS.queryHost("beer-control").toString();
  // Serial.println(ip);
  return ip;
}

void WIFI::startAPMode(){
  Serial.println("\nFailed to connect to WiFi. Starting AP mode...");
  WiFi.softAP(hostname, NULL);  // Start the AP without a password

  // Start server for configuration page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", SETUP_WIFI_HTML);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    String new_ssid;
    String new_pass;

    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
      new_ssid = request->getParam("ssid", true)->value();
      new_pass = request->getParam("password", true)->value();
      
      // Save the new credentials to SPIFFS
      saveCredentials(new_ssid, new_pass);
      
      request->send(200, "text/plain", "Credentials saved. The ESP32 will now reboot.");
      delay(3000); // Give some time to send the response
      ESP.restart();
    } else {
      request->send(400, "text/plain", "Invalid request. Please submit SSID and password.");
    }
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/css", STYLE_CSS);
  });

  server.begin();
}

void WIFI::connectToWiFi(){
  WiFi.begin(ssid, password);
  uint32_t notConnectedCounter = 0;
  EEPROM.begin(32);
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    logger.println("Wifi connecting...");
      
    notConnectedCounter++;
    if(notConnectedCounter > 7) { // Reset board if not connected after 5s
      logger.println("Resetting due to Wifi not connecting...");
      const uint8_t num_of_tries = EEPROM.readInt(1);
      if (num_of_tries == 3) {
        // startAPMode(); // not thinked yet how implement this but is workin
        break; 
      }         
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

  logger.printValue("IP address: ", String(WiFi.localIP().toString()));
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



