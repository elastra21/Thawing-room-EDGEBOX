#ifndef MY_MQTT_H
#define MY_MQTT_H
#include <Arduino.h>
#include <PubSubClient.h>
#include "hardware/logger.h"
#include <WiFiClientSecure.h>

#define MQTT_USERNAME_SIZE 32


//             subscribe topics    -------------------------------------------------------------------->
#define sub_hours           "mfp1/hours"
#define sub_minutes         "mfp1/minutes"
#define sub_day             "mfp1/day"
#define sub_month           "mfp1/month"
#define sub_f1_st1_ontime   "mfp1/f1_st1_ontime"
#define sub_f1_st1_offtime  "mfp1/f1_st1_offtime"
#define sub_f1_st2_ontime   "mfp1/f1_st2_ontime"
#define sub_f1_st2_offtime  "mfp1/f1_st2_offtime"
#define sub_s1_st2_ontime   "mfp1/s1_st2_ontime"
#define sub_s1_st2_offtime  "mfp1/s1_st2_offtime"
#define sub_f1_st3_ontime   "mfp1/f1_st3_ontime"
#define sub_f1_st3_offtime  "mfp1/f1_st3_offtime"
#define sub_s1_st3_ontime   "mfp1/s1_st3_ontime"
#define sub_s1_st3_offtime  "mfp1/s1_st3_offtime"
#define sub_A               "mfp1/A"
#define sub_B               "mfp1/B"
#define sub_ts_set          "mfp1/ts_set"
#define sub_tc_set          "mfp1/tc_set"
#define sub_start           "mfp1/start"
#define sub_d_start         "mfp1/d_start"
#define sub_stop            "mfp1/stop"
#define sub_TempAcqDelay    "mfp1/TempAcqDelay"
#define sub_P               "mfp1/P"
#define sub_I               "mfp1/I"
#define sub_D               "mfp1/D"
#define sub_avgTiming       "mfp1/TsAvgTime"      // in ms the sampling rate for Ts calculation
#define sub_tsAvgSpan       "mfp1/TsAvgFifoSpan"  // in minutes the span of the fifo for Ts calculation
#define sub_chooseTs        "mfp1/chooseTs"
#define sub_coefPID         "mfp1/coefPID"

//------------ publish index    -------------------------------------------------------------------->
#define m_F1                "mfp1/M_F1"
#define m_F2                "mfp1/M_F2"
#define m_S1                "mfp1/M_S1"
#define STAGE               "mfp1/stage"
#define AVG_TS_TOPIC        "mfp1/AvgTs"
#define TA_TOPIC            "mfp1/Ta"
#define TS_TOPIC            "mfp1/Ts"
#define TC_TOPIC            "mfp1/Tc"
#define TI_TOPIC            "mfp1/Ti"
#define PID_OUTPUT          "mfp1/PID_output"
#define SETPOINT            "mfp1/setpoint"
#define ACK_F1_ST1_ONTIME   "mfp1/ack_f1_st1_ontime"
#define ACK_F1_ST1_OFFTIME  "mfp1/ack_f1_st1_offtime"
#define ACK_F1_ST2_ONTIME   "mfp1/ack_f1_st2_ontime"
#define ACK_F1_ST2_OFFTIME  "mfp1/ack_f1_st2_offtime"
#define ACK_S1_ST2_ONTIME   "mfp1/ack_s1_st2_ontime"
#define ACK_S1_ST2_OFFTIME  "mfp1/ack_s1_st2_offtime"
#define ACK_F1_ST3_ONTIME   "mfp1/ack_f1_st3_ontime"
#define ACK_F1_ST3_OFFTIME  "mfp1/ack_f1_st3_offtime"
#define ACK_S1_ST3_ONTIME   "mfp1/ack_s1_st3_ontime"
#define ACK_S1_ST3_OFFTIME  "mfp1/ack_s1_st3_offtime"
#define ACK_A               "mfp1/ack_A"
#define ACK_B               "mfp1/ack_B"
#define ACK_TS              "mfp1/ack_Ts"
#define ACK_TC              "mfp1/ack_Tc"
#define SPOILED_SENSOR      "mfp1/spoiled_sensor"



class MqttClient {
  public:
    void loop();
    void connect(const char *domain, uint16_t port, const char *username);
    void reconnect();
    bool isConnected();
    void subscribeRoutine();
    bool refreshMQTTStatus();
    bool isServiceAvailable();
    bool getConnectionStatus();
    void publishData(String topic, double value);
    void publishData(String topic, String value);
    bool isTopicEqual(const char* a, const char* b);
    void setCallback(std::function<void (char *, uint8_t *, unsigned int)> callback);
    void publishEcava(const String* topics, const String* values, int arraySize, const char* mqttTopic);
    String getIsoTimestamp();
    void exampleCall();
  private:
    String prefix = "mduino/";
    uint16_t mqtt_port;
    char mqtt_username[MQTT_USERNAME_SIZE];  
    char mqtt_domain[MQTT_USERNAME_SIZE];
    bool no_service_available = true;
    bool last_connection_state = false;
    String getTopic(const char* topic);
};
#endif
