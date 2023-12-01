#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <EdgeBox_ESP_100.h>

#define SECS_IN_HR 3600
// #define TIME_ZONE_OFFSET_HRS            (-7)  /* Ensenada, México */
// // #define TIME_ZONE_OFFSET_HRS            (+8)   /* Taiping, Malaysia */

// temperature acquisition filter 
#define HIGH_TEMP_LIMIT 60
#define LOW_TEMP_LIMIT -40

// setting PWM properties
#define AIR_PWM     0       // Sí me mamé pero aún así es sólo el canal no el pin
#define FREQ        5000 
#define AIR_PIN     AO0     // Este es el pin que va tener controlando el PWM
#define RESOLUTION  8    

//------------ IO's    -------------------------------------------------------------------->
#define STAGE_1_IO  DO_0   
#define STAGE_2_IO  DO_1   
#define STAGE_3_IO  DO_2   
#define VALVE_IO    DO_3   
#define FAN_IO      DO_4   

// #define A0_5  27                            

// #define STOP_IO     DI_0    
// #define DLY_S_IO    DI_1    
// #define START_IO    DI_2    

#define STOP_IO     DI_2    
#define DLY_S_IO    DI_1    
#define START_IO    DI_0  

#define A0    13 //ONE_WIRE_BUS  

#define TA_AI       0
#define TS_AI       1
#define TC_AI       2

#define BUFFER_SIZE 60 

// Temperature Sensors settings          -------------------------------------------------------------------->
#define ONE_WIRE_BUS 13// Data wire is plugged into port 2 on the Arduin
#define TEMPERATURE_PRECISION 12

#define TIME_ACQ_DELAY 1000 //in ms the delay between temperature value refresh
#define AVG_RESOLUTION 1000   //in ms the sampling for the Ts measure

#define IR_SENSOR_ADDRESS 0x5A
#define READ_TEMPERATURE 0x07

#endif