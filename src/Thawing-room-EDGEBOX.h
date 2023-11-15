#ifndef THAWING_ROOM_H
#define THAWING_ROOM_H

#include <FS.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <PID_v1.h>
// #include "secrets.h"
#include <Arduino.h>
#include "MqttClient.h"
#include <ArduinoJson.h>
#include "hardware/logger.h"
#include "hardware/config.h"
#include "hardware/Controller.h"

//---- Function declaration ----/////////////////////////////////////////////////////////////////////////////
void setUpRTC();
float getIRTemp();
void stopRoutine();
void updateTemperature();
void setStage(int Stage);
void setUpDefaultParameters();
void updateDefaultParameters();
String addressToString(uint8_t *address);
int responseToInt(byte *value, size_t len);
float responseToFloat(byte *value, size_t len);
void sendTemperaturaAlert(float temp, String sensor);
bool validateTemperature(float temp, uint8_t type);
void callback(char *topic, byte *payload, unsigned int len);  //callback function for mqtt, see definition after loop
void runConfigFile(char* ssid, char* password, char* hostname, char* ip_address, uint16_t* port, char* username);


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

//------------ structure definitions an flags -------------------------------------------------------->
// Fan F1 and sprinkler S1 value
typedef struct { float M_F1; }                            data_F1;

typedef struct { float M_F2; }                            data_F2;

typedef struct { float M_S1; }                            data_S1;

//stage
typedef struct { float stage; }                           data_stage;

// A and B variables
typedef struct { float N_A; float N_B; }                  data_SP;

// PID variables
typedef struct { float PID_output; }                      data_PIDO;

typedef struct { float PID_setpoint; }                    data_setpoint;

typedef struct { float N_P; float N_I; float N_D; }       data_PID;

// Ts and Tc target value
typedef struct { float N_ts_set; float N_tc_set; }        data_tset;

// fan (F1) STAGE 1 on and off time 
typedef struct { float N_f1_st1_ontime; float N_f1_st1_offtime; }                 data_st1;

// RTC
typedef struct { float N_hours; float N_minutes; float N_day; float N_month; }    data_rtc;

// temperature measures
typedef struct { float Ta_N; float Ts_N; float Tc_N; float Ti_N; float AvgTs_N; } data_s;


// fan (F1) and sprinklers (S1) STAGE 2 on and off time 
typedef struct { float N_f1_st2_ontime; float N_f1_st2_offtime; float N_s1_st2_ontime; float N_s1_st2_offtime; } data_st2;

// fan (F1) and sprinklers (S1) STAGE 3 on and off time 
typedef struct { float N_f1_st3_ontime; float N_f1_st3_offtime; float N_s1_st3_ontime; float N_s1_st3_offtime; } data_st3;

enum SensorProbes{TA_TYPE, TS_TYPE, TC_TYPE};

#endif