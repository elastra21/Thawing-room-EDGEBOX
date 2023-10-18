#ifndef THAWING_ROOM_H
#define THAWING_ROOM_H

#include <Arduino.h>

//---- Function declaration ----/////////////////////////////////////////////////////////////////////////////
void setUpRTC();
float getIRTemp();
void stopRoutine();
void updateTemperature();
void setStage(int Stage);
void setUpDefaultParameters();
String addressToString(uint8_t *address);
int responseToInt(byte *value, size_t len);
float responseToFloat(byte *value, size_t len);
void callback(char *topic, byte *payload, unsigned int len);  //callback function for mqtt, see definition after loop


//---- timing settings -----////////////////////////////////////////////////////////////////////////////////

#define MINS 60000

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


#endif