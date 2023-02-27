#include <DallasTemperature.h>

// #define TIME_ZONE_OFFSET_HRS            (-7)  /* Ensenada, México */
#define TIME_ZONE_OFFSET_HRS            (+8)   /* Taiping, Malaysia */

// temperature acquisition filter 
#define HIGH_TEMP_LIMIT 60
#define LOW_TEMP_LIMIT -40

// setting PWM properties
#define AIR_PWM 27
#define FREQ 5000
#define AIR_PIN 0
#define RESOLUTION 8

//------------ IO's    -------------------------------------------------------------------->
#define Q0_0  14  // Stage #1
#define Q0_1  25  // Stage #2
#define Q0_2  26  // Stage #3
#define Q0_3  0  // Valve
#define Q0_4  19  // Fan

#define A0_5  27  //                          *

#define I0_10 2  // Stop button
#define I0_11 33   // Delayed start button     *
#define I0_12 34  // Start button

#define A0    13 //ONE_WIRE_BUS  

#define BUFFER_SIZE 60 

// Temperature Sensors settings          -------------------------------------------------------------------->
#define ONE_WIRE_BUS 13// Data wire is plugged into port 2 on the Arduin
#define TEMPERATURE_PRECISION 12

#define TIME_ACQ_DELAY 10000 //in ms the delay between temperature value refresh
#define AVG_RESOLUTION 1000   //in ms the sampling for the Ts measure

DeviceAddress ADDRESS_TC1 = { 0x28, 0x8C, 0x4B, 0xAD, 0x27, 0x19, 0x01, 0xCA }; // Ta
// DeviceAddress ADDRESS_TC1 = { 0x28, 0xA7, 0x93, 0x8B, 0x0B, 0x00, 0x00, 0xB2 }; // Ta
DeviceAddress ADDRESS_TC2 = { 0x28, 0x78, 0x98, 0x8B, 0x0B, 0x00, 0x00, 0x22 }; // Ts
DeviceAddress ADDRESS_TC3 = { 0x28, 0xDA, 0xB6, 0xF7, 0x3A, 0x19, 0x01, 0x85 }; // Tc 
DeviceAddress ADDRESS_TC4 = { 0x28, 0x5A, 0xD3, 0x2A, 0x0D, 0x00, 0x00, 0x94 }; // Ti

#define SENSOR_ADDRESS 0x5A
#define READ_TEMPERATURE 0x07

//---- timing settings -----////////////////////////////////////////////////////////////////////////////////

#define MINS 60000

//------------ structure definitions an flags -------------------------------------------------------->
// Fan F1 and sprinkler S1 value
typedef struct { float M_F1; }                            data_F1;

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
