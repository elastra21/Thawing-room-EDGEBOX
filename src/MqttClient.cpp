#include "hardware/Controller.h"  // This is included in order that the compiler knows the type of the variable logger
#include "MqttClient.h"

WiFiClient esp32Client;
PubSubClient mqttClient(esp32Client);

// void subscribeReceive(char* topic, byte* payload, unsigned int length);

void MqttClient::connect(const char *domain, uint16_t port, const char *id, const char *username, const char *password) {
  //storing the username, port and domain in a global variable
  strncpy(mqtt_domain, domain, sizeof(mqtt_domain) - 1);
  mqtt_domain[sizeof(mqtt_domain) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  mqtt_port = port;

  strncpy(mqtt_id, id, sizeof(mqtt_id) - 1);
  mqtt_id[sizeof(mqtt_id) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(mqtt_username, username, sizeof(mqtt_username) - 1);
  mqtt_username[sizeof(mqtt_username) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(mqtt_password, password, sizeof(mqtt_password) - 1);
  mqtt_password[sizeof(mqtt_password) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  Serial.println("Domain: " + String(domain));
  Serial.println("Port: " + String(port));
  Serial.println("ID: " + String(mqtt_id));
  Serial.println("Username: " + String(mqtt_username));
  Serial.println("Password: " + String(mqtt_password));

  mqttClient.setServer(domain, port);
  if (mqttClient.connect(mqtt_id, mqtt_username, mqtt_password)) {
    logger.println("Connection has been established, well done");
    subscribeRoutine();
    no_service_available = false;
  } else {
    logger.println("Looks like the server connection failed...");
  }
}

bool MqttClient::isServiceAvailable() {
  return !no_service_available;
}


void MqttClient::reconnect() {
  static unsigned long lastReconnectAttempt = 0;
  unsigned long now = millis();
  static int reconnectAttempts = 0;

  if (!mqttClient.connected()) {
    if (now - lastReconnectAttempt > 120000 || reconnectAttempts == 0) { // 120000ms = 2 minutos
      lastReconnectAttempt = now;

      if (reconnectAttempts < 5) {
        mqttClient.flush();
        mqttClient.disconnect();
        mqttClient.setServer(mqtt_domain, mqtt_port);
        
        WebSerial.print("Attempting MQTT connection...");

        if (mqttClient.connect(mqtt_id, mqtt_username, mqtt_password)) {
          WebSerial.println("connected");
          subscribeRoutine();
          reconnectAttempts = 0; // Resetear los intentos si la conexión es exitosa
        } else {
          WebSerial.print("failed, rc=");
          WebSerial.print(mqttClient.state());
          WebSerial.println(" try again in 2 minutes");
          reconnectAttempts++;
        }
      } else {
        WebSerial.println("Max reconnect attempts reached, try again in 2 minutes");
        reconnectAttempts = 0; // Resetear los intentos después de alcanzar el máximo
      }
    }
  } else {
    reconnectAttempts = 0; // Resetear los intentos si ya está conectado
  }
}

bool MqttClient::isTopicEqual(const char* a, const char* b){
  return strcmp(a, b) == 0;
}

bool MqttClient::isConnected() {
  return mqttClient.connected();
}

void MqttClient::loop() {
  if (!isServiceAvailable()) return;
  if (!mqttClient.connected()) reconnect();

  vTaskDelay(100 / portTICK_PERIOD_MS);
  mqttClient.loop();
}

void MqttClient::setCallback(std::function<void(char *, uint8_t *, unsigned int)> callback) {
  mqttClient.setCallback(callback);
}

void MqttClient::subscribeRoutine() {
  if (mqttClient.connect(mqtt_id, mqtt_username, mqtt_password)) {
    logger.println("connected, subscribing");
    if (!mqttClient.subscribe(sub_hours, 1)) logger.println("sub hours failed !");
    if (!mqttClient.subscribe(sub_minutes, 1)) logger.println("sub hours failed !");
    if (!mqttClient.subscribe(sub_day, 1)) logger.println("sub hours failed !");
    if (!mqttClient.subscribe(sub_month, 1)) logger.println("sub hours failed !");
    if (!mqttClient.subscribe(sub_f1_st1_ontime, 1)) logger.println("sub hours failed !");
    if (!mqttClient.subscribe(sub_f1_st1_offtime, 1)) logger.println("sub hours failed !");
    if (!mqttClient.subscribe(sub_f1_st2_ontime, 1)) logger.println("sub hours failed !");
    mqttClient.subscribe(sub_f1_st2_offtime, 1);
    mqttClient.subscribe(sub_s1_st2_ontime, 1);
    mqttClient.subscribe(sub_s1_st2_offtime, 1);
    mqttClient.subscribe(sub_f1_st3_ontime, 1);
    mqttClient.subscribe(sub_f1_st3_offtime, 1);
    mqttClient.subscribe(sub_s1_st3_ontime, 1);
    mqttClient.subscribe(sub_s1_st3_offtime, 1);
    mqttClient.subscribe(sub_A, 1);
    if (!mqttClient.subscribe(sub_B, 1)) logger.println("sub hours failed !");
    mqttClient.subscribe(sub_P, 1);
    mqttClient.subscribe(sub_I, 1);
    mqttClient.subscribe(sub_D, 1);
    mqttClient.subscribe(sub_tc_set, 1);
    mqttClient.subscribe(sub_ts_set, 1);
    mqttClient.subscribe(sub_start, 1);
    mqttClient.subscribe(sub_d_start, 1);
    mqttClient.subscribe(sub_stop, 1);
    mqttClient.subscribe(sub_stop, 1);
    mqttClient.subscribe(sub_avgTiming, 1);
    mqttClient.subscribe(sub_tsAvgSpan, 1);
    mqttClient.subscribe(sub_chooseTs, 1);
    mqttClient.subscribe(sub_coefPID, 1);
    // mqttClient.subscribe(sub_address1, 1);
    // mqttClient.subscribe(sub_address2, 1);
    // mqttClient.subscribe(sub_address3, 1);
    // mqttClient.subscribe(sub_address4, 1);
    logger.println("subscribing done");
  } else logger.println("not connected, subscribing aborted");
}

void MqttClient::publishData(String topic, double value) {
  if (WiFi.status() != WL_CONNECTED) return;
  char value_buffer[8];
  dtostrf(value, 1, 2, value_buffer);
  mqttClient.publish(topic.c_str(), value_buffer);
}

void MqttClient::publishData(String topic, String value) {
  if (WiFi.status() != WL_CONNECTED) return;
  mqttClient.publish(topic.c_str(), value.c_str());
}

bool MqttClient::refreshMQTTStatus() {
  const bool connection = isConnected();
  if (connection != last_connection_state) {
    last_connection_state = connection;
    return true;
  }
  return false;
}

bool MqttClient::getConnectionStatus() {
  return last_connection_state;
}

// Función que construye el JSON y lo publica.
void MqttClient::publishEcava(const String* topics, const String* values, int arraySize, const char* mqttTopic) {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<512> jsonDoc; // Adjust the size as necessary

  String timeStamp = getIsoTimestamp(); // Ensure you have a function to get the ISO timestamp.

  for (int i = 0; i < arraySize; i++) {
    // This will create a nested array for each topic
    JsonArray nestedArray = jsonDoc.createNestedArray(topics[i]);
    // Now we create a nested array for the timestamp and value pair
    JsonArray dataPair = nestedArray.createNestedArray();
    dataPair.add(timeStamp);
    dataPair.add(values[i]);
  }

  String output;
  serializeJson(jsonDoc, output);
  mqttClient.publish(mqttTopic, output.c_str());
}

// Función para obtener el timestamp ISO 8601.
// Deberás implementar esto para que funcione con tu configuración específica y zona horaria.
String MqttClient::getIsoTimestamp() {
  // Implementa la obtención del timestamp aquí.
  // Devuelve el timestamp en formato ISO 8601 como String.
  return "2023-11-13T12:49:52.737+08:00";
}

String MqttClient::getTopic(const char* topic){
  return prefix + topic;
}

// Función de ejemplo de cómo podrías llamar a publishData.
void MqttClient::exampleCall() {
  const String topics[] = {"mfp1.Tc", "mfp1.Ta", "mfp1.Ts"};
  const String values[] = {"23.5", "22.5", "21.5"};

  publishEcava(topics, values, 3, "xda.json://Fish.dxscript/Fish/onchange");
}

float MqttClient::responseToFloat(byte *value, size_t len) {
  String string_builder;
  for (int i = 0; i < len; i++) string_builder += (char)value[i];
  return string_builder.toFloat();
}

int MqttClient::responseToInt(byte *value, size_t len) {
  String string_builder;
  for (int i = 0; i < len; i++) string_builder += (char)value[i];
  return string_builder.toInt();
}