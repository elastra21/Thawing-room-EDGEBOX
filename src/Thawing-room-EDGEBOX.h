#ifndef THAWING_ROOM_H
#define THAWING_ROOM_H

#include <Wire.h>
#include <PID_v1.h>
// #include "secrets.h"
#include <Arduino.h>
#include "MqttClient.h"
#include "hardware/logger.h"
#include "hardware/config.h"
#include "hardware/Controller.h"

//------------ structure definitions an flags -------------------------------------------------------->

// temperature measures
typedef struct { float ta; float ts; float tc; float ti; float avg_ts; } data_s;

enum SensorProbes{TA_TYPE, TS_TYPE, TC_TYPE};

//---- Function declaration ----/////////////////////////////////////////////////////////////////////////////
// float getIRTemp();
void getTsAvg();
void publishPID();
void stopRoutine();
void handleStage1();
void handleStage2();
void handleStage3();
void handleInputs();
bool noButtonPressed();
void updateTemperature();
void aknowledgementRoutine();
void setStage(SystemState Stage);
bool shouldStage2Start(DateTime &current_date);
bool shouldStage3Start(DateTime &current_date);
void publishTemperatures(DateTime &current_date);
void sendTemperaturaAlert(float temp, String sensor);
void callback(char *topic, byte *payload, unsigned int len);  //callback function for mqtt, see definition after loop
void publishStateChange(const char* topic, int state, const String& message);
bool hasIntervalPassed(uint32_t &previousMillis, uint32_t interval, bool to_min = false);
bool isValidTemperature(float temp, float minTemp, float maxTemp, const String& sensorName);


//---- timing settings -----////////////////////////////////////////////////////////////////////////////////

#define MINS 60000

// ---- Probes min and max values ----//////////////////////////////////////////////////////////////////////// 

#define TA_MIN -5
#define TA_MAX 25
#define TA_DEF 15 

#define TS_MIN -20
#define TS_MAX 10
#define TS_DEF 5

#define TC_MIN -20
#define TC_MAX 5
#define TC_DEF -1


#endif