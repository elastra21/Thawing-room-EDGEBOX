#include "hardware/Controller.h"  // This is included in order that the compiler knows the type of the variable logger
#include "MqttClient.h"

WiFiClient esp32Client;
PubSubClient mqttClient(esp32Client);

// void subscribeReceive(char* topic, byte* payload, unsigned int length);

void MqttClient::connect(const char *domain, uint16_t port, const char *username) {
  strncpy(mqtt_username, username, sizeof(mqtt_username) - 1);
  mqtt_username[sizeof(mqtt_username) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  mqttClient.setServer(domain, port);
  if (mqttClient.connect(mqtt_username)) {
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
  while (!mqttClient.connected()) {
    logger.print("Attempting MQTT connection...");
    if (mqttClient.connect(mqtt_username)) {
      logger.println("connected");
      subscribeRoutine();
    } else {
      logger.print("failed, rc=");
      logger.printValue("failed, rc=",String(mqttClient.state()));
      logger.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

bool MqttClient::isConnected() {
  return mqttClient.connected();
}

void MqttClient::loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  delay(100);
  mqttClient.loop();
}

void MqttClient::setCallback(std::function<void(char *, uint8_t *, unsigned int)> callback) {
  mqttClient.setCallback(callback);
}

void MqttClient::subscribeRoutine() {
  if (mqttClient.connect(mqtt_username)) {
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
  char value_buffer[8];
  dtostrf(value, 1, 2, value_buffer);
  mqttClient.publish(topic.c_str(), value_buffer);
}

void MqttClient::publishData(String topic, String value) {
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
