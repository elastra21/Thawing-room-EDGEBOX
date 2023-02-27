#include <SPI.h>
#include <Wire.h>
#include "WIFI.h"
#include "RTClib.h"
#include "config.h"
#include "Screen.h"
#include <PID_v1.h>
#include <OneWire.h>
#include <M5Tough.h>
#include <WiFiUdp.h>
#include "secrets.h"
#include "MqttClient.h"
#include <DallasTemperature.h>
#include <NTPClient_Generic.h>

RTC_DateTypeDef RTC_DateStruct;  // Data
RTC_TimeTypeDef RTC_TimeStruct;  // Time

data_rtc N_rtc;  // structure data_rtc from the config file is renamed N_rtc
data_st1 N_st1;  // fan (F1) STAGE 1 on and off time
data_st2 N_st2;  // fan (F1) and sprinklers (S1) STAGE 2 on and off time
data_st3 N_st3;  // fan (F1) and sprinklers (S1) STAGE 3 on and off time

// State of SPRINKLER 1
bool S1_state = 0;

// A & B variables
data_SP N_SP;
float A = 0;
float B = 0;
bool R_A = 0;
bool R_B = 0;

// PID variables
data_PIDO PID_data;           // value of the PID output
data_setpoint setpoint_data;  // value of the Setpoint

bool R_P = 0;
bool R_I = 0;
bool R_D = 0;

double Output;    // PWM signal and converter
double PIDinput;  // temp sensor
double Setpoint;  // will be the desired value

// PID parameters
double Kp = 0, Ki = 10, Kd = 0;

double coefOutput = 0;  // Output for the infeed (New Analog Output that will be sent to S1
uint8_t coefPID = 100;
float Output_float = 0.0;
uint8_t Converted_Output = 0;

// Temperatures measures of Ta, Ts, Tc, Ti & AvgTs
data_s temp_data;

// Ts & Tc target value
data_tset N_tset;
bool stop_temp1 = 0;
bool stop_temp2 = 0;

// Fan F1 value (1 parameter)
data_F1 F1_data;

// Sprinkler S1 value (1 parameter)
data_S1 S1_data;

// Start, delayed start, stop, and choose Ts
uint8_t N_stop = 0;
uint8_t N_start = 0;
uint8_t N_d_start = 0;
uint8_t N_chooseTs = 0;

bool STOP = 0;
bool START = 0;

bool START1 = 0;     // delayed start bttn
bool START2 = 0;     // start bttn
bool C1_state = 0;   // State of Stage 1
bool C2_state = 0;   // State of Stage 2
bool C3_state = 0;   // State of Stage 3
bool MTR_State = 0;  // State of the motor that control the Fan F1

// State of the Stage (data = 1, 2 or 3)
data_stage stage_data;
uint8_t stage = 0;

// Parameters of Stage 2
uint8_t Stage2_hour = 0;
uint8_t Stage2_minute = 0;
uint8_t Stage2_day = 0;
uint8_t Stage2_month = 0;

bool Stage2_RTC_set = 0;
bool Stage2_started = 0;
bool Stage3_started = 0;

//---- timing variable -----////////////////////////////////////////////////////////////////////////////////
uint32_t F1_timer = 0UL;               // fan F1 timing
uint32_t pid_computing_timer = 0UL;    // PID computing timing
uint32_t F1_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t S1_stg_2_timer = 0UL;         // S1 stage 2 timing
uint32_t F1_stg_3_timer = 0UL;         // F1 stage 3 timing
uint32_t S1_stg_3_timer = 0UL;         // S1 stage 3 timing
uint32_t get_temp_timer = 0UL;         // temperature acquisition
uint32_t ts_avg_timer = 0UL;           // Ts average timing
uint32_t stg_2_pid_timer = 0UL;        // stage 2 PID
uint32_t turn_on_pid_timer = 0UL;      // stage 2 PID ON
uint32_t turn_off_pid_timer = 0UL;     // stage 2 PID OFF
uint32_t address_sending_timer = 0UL;  // Address Sending
uint32_t A_B_timer = 0UL;              // Stage ON/OFF and A&B PUBLISH

float TC1 = 0, TC1_F = 0;  //Ta
float TC2 = 0, TC2_F = 0;  //Ts
float TC3 = 0, TC3_F = 0;  //Tc
float TC4 = 0, TC4_F = 0;  //optionely

float buffer[BUFFER_SIZE] = {};  // buffer to store the values
uint8_t buffer_len = 0;
uint8_t bufferIndex = 0;  // buffer index
float buffer_sum = 0;     // variable to store the buffer_sum of the received values
float AvgTC2 = 0.0;       // average surface temperature

WIFI wifi;
Screen screen;
WiFiUDP ntpUDP;
MqttClient mqtt;

NTPClient timeClient(ntpUDP);

PID air_in_feed_PID(&PIDinput, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // DIRECT or REVERSE

OneWire oneWire(ONE_WIRE_BUS);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors1(&oneWire);  // PASS our oneWire reference to Dallas Temperature.

//---- Function declaration ----/////////////////////////////////////////////////////////////////////////////
void setUpRTC();
float getIRTemp();
void stopRoutine();
void updateTemperature();
void setStage(int Stage);
String addressToString(uint8_t *address);
void callback(char *topic, byte *payload, unsigned int len);  //callback function for mqtt, see definition after loop

void setup() {
  M5.begin();
  Wire.begin();
  M5.Lcd.setRotation(3);
  screen.printLabels();

  setStage(0);
  screen.updateIndicators(0, 0, 0);

  pinMode(Q0_0, OUTPUT);
  pinMode(Q0_1, OUTPUT);
  pinMode(Q0_2, OUTPUT);
  pinMode(Q0_3, OUTPUT);
  pinMode(Q0_4, OUTPUT);

  ledcSetup(AIR_PWM, FREQ, RESOLUTION);
  ledcAttachPin(AIR_PIN, AIR_PWM);

  pinMode(I0_10, INPUT);
  // pinMode(I0_11, INPUT);
  pinMode(I0_12, INPUT);

  wifi.setUpWiFi();
  setUpRTC();

  mqtt.connect();
  mqtt.setCallback(callback);

  //Turn the PID on
  air_in_feed_PID.SetMode(AUTOMATIC);
  air_in_feed_PID.SetSampleTime(3000);
  //Adjust PID values
  air_in_feed_PID.SetTunings(Kp, Ki, Kd);
  sensors1.setResolution(ADDRESS_TC1, TEMPERATURE_PRECISION);
  sensors1.setResolution(ADDRESS_TC2, TEMPERATURE_PRECISION);
  sensors1.setResolution(ADDRESS_TC3, TEMPERATURE_PRECISION);
  sensors1.setResolution(ADDRESS_TC4, TEMPERATURE_PRECISION);
  delay(750);
}

void loop() {
  mqtt.loop();

  updateTemperature();

  // if (N_chooseTs == 1) TC2 = analogRead(A0);                                // Condition to choose if Ts is a IR sensor or OneWire sensor

  if ((TC1) > (LOW_TEMP_LIMIT) && (TC1) < (HIGH_TEMP_LIMIT)) TC1_F = TC1;  // if the temperature over the limit it will not be considered

  if ((TC2) > (LOW_TEMP_LIMIT) && (TC2) < (HIGH_TEMP_LIMIT)) TC2_F = TC2;

  if ((TC3) > (LOW_TEMP_LIMIT) && (TC3) < (HIGH_TEMP_LIMIT)) TC3_F = TC3;

  if ((TC4) > (LOW_TEMP_LIMIT) && (TC4) < (HIGH_TEMP_LIMIT)) TC4_F = TC4;

  if ((millis() - address_sending_timer >= 10000)) {

    String S_add1 = addressToString(ADDRESS_TC1);
    mqtt.publishData("mduino/sendadd1", S_add1);

    String S_add2 = addressToString(ADDRESS_TC2);
    mqtt.publishData("mduino/sendadd2", S_add2);

    String S_add3 = addressToString(ADDRESS_TC3);
    mqtt.publishData("mduino/sendadd3", S_add3);

    String S_add4 = addressToString(ADDRESS_TC4);
    mqtt.publishData("mduino/sendadd4", S_add4);

    address_sending_timer = millis();
  }

  //---- Get surface temperature average with a FIFO buffer ---- //////////////////////////////// Something fuckin' wrong with the average
  if (millis() - ts_avg_timer >= AVG_RESOLUTION) {
    const bool full_buffer = !(buffer_len < BUFFER_SIZE);
    const uint8_t index = full_buffer ? bufferIndex : buffer_len;

    buffer_sum += TC2_F;
    buffer[index] = TC2_F;

    if (full_buffer) {
      buffer_sum -= buffer[index];
      bufferIndex = (index + 1) % BUFFER_SIZE;
    } else buffer_len++;

    AvgTC2 = buffer_sum / buffer_len;

    mqtt.publishData(AVG_TS, temp_data.AvgTs_N);
    Serial.println("Temp data published");
    ts_avg_timer = millis();
  }

  //---- Temperature MQTT publish ----///////////////////////////////////////////////////////////
  if (millis() - get_temp_timer >= TIME_ACQ_DELAY) {
    temp_data.Ta_N = TC1_F;
    temp_data.Ts_N = TC2_F;
    temp_data.Tc_N = TC3_F;
    temp_data.Ti_N = TC4_F;
    temp_data.AvgTs_N = AvgTC2;

    mqtt.publishData(TA, temp_data.Ta_N);
    mqtt.publishData(TS, temp_data.Ts_N);
    mqtt.publishData(TC, temp_data.Tc_N);
    mqtt.publishData(TI, temp_data.Ti_N);

    // for debug purpose
    Serial.println("Average: " + String(temp_data.AvgTs_N));
    Serial.println("Ts: " + String(TC2));
    Serial.println("TC: " + String(TC3));
    Serial.println("Ta: " + String(TC1));
    Serial.println("Nstart: " + String(N_start));
    Serial.println("Nstop: " + String(N_stop));
    Serial.println("A variable: " + String(N_SP.N_A));
    Serial.println("B variable: " + String(N_SP.N_B));
    Serial.println("P variable: " + String(Kp));
    Serial.println("I variable: " + String(Ki));
    Serial.println("D variable: " + String(Kd));
    Serial.println("setpoint raw: " + String(Setpoint));
    Serial.println("setpoint: " + String(setpoint_data.PID_setpoint));

    Serial.println("time: ");
    Serial.print(String(RTC_TimeStruct.Hours) + "h ");
    Serial.println(String(RTC_TimeStruct.Minutes) + "min ");
    Serial.print(String(RTC_DateStruct.Date) + "day ");
    Serial.println(String(RTC_DateStruct.Month) + "month");
    Serial.println("stage 2 time: ");
    Serial.print(String(Stage2_hour) + "h ");
    Serial.print(String(Stage2_minute) + "min ");
    Serial.print(String(Stage2_day) + "day ");
    Serial.print(String(Stage2_month) + "month");

    get_temp_timer = millis();
  }

  //---- Time Stage ON/OFF and A & B MQTT Publish ----///////////////////////////////////////////////////////
  if (millis() - A_B_timer >= 10000) {
    // STAGE 1
    mqtt.publishData(ACK_F1_ST1_ONTIME, N_st1.N_f1_st1_ontime);
    mqtt.publishData(ACK_F1_ST1_OFFTIME, N_st1.N_f1_st1_offtime);

    // STAGE 2
    mqtt.publishData(ACK_F1_ST2_ONTIME, N_st2.N_f1_st2_ontime);
    mqtt.publishData(ACK_F1_ST2_OFFTIME, N_st2.N_f1_st2_offtime);
    mqtt.publishData(ACK_S1_ST2_ONTIME, N_st2.N_s1_st2_ontime);
    mqtt.publishData(ACK_S1_ST2_OFFTIME, N_st2.N_s1_st2_offtime);

    // STAGE 3
    mqtt.publishData(ACK_F1_ST3_ONTIME, N_st3.N_f1_st3_ontime);
    mqtt.publishData(ACK_F1_ST3_OFFTIME, N_st3.N_f1_st3_offtime);
    mqtt.publishData(ACK_S1_ST3_ONTIME, N_st3.N_s1_st3_ontime);
    mqtt.publishData(ACK_S1_ST3_OFFTIME, N_st3.N_s1_st3_offtime);

    // A & B
    mqtt.publishData(ACK_A, N_SP.N_A);
    mqtt.publishData(ACK_B, N_SP.N_B);

    A_B_timer = millis();
  }

  //---- PID Publishing ----//////////////////////////////////////////////////////////////////////
  // PID works only on STAGE 2
  if (stage == 2 && STOP == 0) {
    if (millis() - stg_2_pid_timer >= (TIME_ACQ_DELAY + 1)) {
      Serial.println("Soft PID Actual Output is" + String(Output));
      Output_float = float(coefOutput);
      PID_data.PID_output = ((Output_float - 0) / (255 - 0)) * (100 - 0) + 0;
      Serial.println("PID Output /100 is" + String(PID_data.PID_output));

      mqtt.publishData(PID_OUTPUT, PID_data.PID_output);
      stg_2_pid_timer = millis();
    }
  }

  //---- START, DELAYED, STOP Button pressed ----////////////////////////////////////////////////
  // delayed start push button or digital button pressed
  if (/*digitalRead(I0_11) == 1 ||*/ N_d_start == 1) {
    START1 = 1;
    Serial.println("Delayed Start Pressed");
    N_d_start = 0;
    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("Stage 1 init M_S1 stop published");
  }

  // start push button or digital button pressed
  if (digitalRead(I0_12) == 1 || N_start == 1) {
    START2 = 1;
    Serial.println("Start Pressed");
    N_start = 0;
    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("Stage 1 init M_S1 stop published");
  }

  // stop push button or digital button pressed
  if (digitalRead(I0_10) == 1 || N_stop == 1) {
    STOP = 1;
    Serial.println("Stop Pressed");
    N_stop = 0;
  }

  //---- STOP ROUTINE ----///////////////////////////////////////////////////////////////////////
  if (STOP == 1) stopRoutine();
  //---- RTC Timer ----//////////////////////////////////////////////////////////////////////////

  if (((((RTC_TimeStruct.Hours >= Stage2_hour && RTC_TimeStruct.Minutes >= Stage2_minute
          && RTC_DateStruct.Date >= Stage2_day && RTC_DateStruct.Month >= Stage2_month)
         && START1 == 1)
        || START2 == 1)
       && Stage2_started == 0 && Stage2_RTC_set == 0)) {

    START1 = MTR_State = C1_state = 0;
    digitalWrite(Q0_0, LOW);
    digitalWrite(Q0_1, LOW);
    digitalWrite(Q0_2, LOW);
    digitalWrite(Q0_3, LOW);
    digitalWrite(Q0_4, LOW);

    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("All M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("All M_S1 stop published");

    Serial.println("Stage 2 Initiated wait for 5 secs");
    Stage2_RTC_set = Stage2_started = 1;
    delay(5000);
  }

  //---- STAGE 1 ----////////////////////////////////////////////////////////////////////////////
  if (START1 == 1 && Stage2_RTC_set == 0 && STOP == 0) {
    if (C1_state == 0) {
      digitalWrite(Q0_0, HIGH);  // Turn On the LED of Stage 1
      C1_state = 1;              // State of Stage 1 turned ON
      Serial.println("Stage 1 Started");
      setStage(1);
      Serial.println("Stage 1 Status Send packet ");
      F1_timer = millis() - (N_st1.N_f1_st1_ontime * MINS);
    }

    // Turn ON F1

    if (MTR_State == 0 && (HIGH != digitalRead(Q0_4)) && (millis() - F1_timer >= (N_st1.N_f1_st1_offtime * MINS))) {  // MTR_State is the motor of F1
      digitalWrite(Q0_4, HIGH);                                                                                       // Turn ON F1
      Serial.println("Stage 1 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("Stage 1 init M_F1 ON published ");
      F1_timer = millis();
    }

    // Turn OFF F1 when the time set in the configuration is over
    if (MTR_State == 1 && (LOW != digitalRead(Q0_4)) && (millis() - F1_timer >= (N_st1.N_f1_st1_ontime * MINS))) {
      digitalWrite(Q0_4, LOW);
      // ledcWrite(AIR_PWM, 0);
      Serial.println("Stage 1 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("Stage 1 init M_F1 OFF published ");
      F1_timer = millis();
    }
  }

  //---- STAGE 2 ----////////////////////////////////////////////////////////////////////////////
  if (Stage2_RTC_set == 1 && Stage3_started == 0 && STOP == 0) {
    if (C2_state == 0) {
      digitalWrite(Q0_1, HIGH);  // Turn On the LED of Stage 2

      C2_state = 1;
      Serial.println("Stage 2 Started");
      stage = 2;
      setStage(2);
      Serial.println("Stage 0 Status Send packet ");
      F1_stg_2_timmer = millis() - (N_st2.N_f1_st2_offtime * MINS);
    }

    // Turn ON F1 when time is over
    if (MTR_State == 0 && (millis() - F1_stg_2_timmer >= (N_st2.N_f1_st2_offtime * MINS))) {
      digitalWrite(Q0_4, HIGH);  // Output of F1
      Serial.println("Stage 2 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stg2 F1 Start published ");
      F1_stg_2_timmer = millis();
    }

    // Turn OFF F1 when time is over
    if (MTR_State == 1 && (millis() - F1_stg_2_timmer >= (N_st2.N_f1_st2_ontime * MINS))) {
      digitalWrite(Q0_4, LOW);
      Serial.println("Stage 2 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stg2 F1 stop published ");
      F1_stg_2_timmer = millis();
    }

    // Turn ON S1 when time is over
    if ((MTR_State == 1) && (S1_state == 0) && (millis() - S1_stg_2_timer >= (N_st2.N_s1_st2_offtime * MINS))) {
      digitalWrite(Q0_3, HIGH);  // Output of S1
      S1_state = 1;
      Serial.println("Stage 2 S1 ON");
      S1_data.M_S1 = 1;  // When M_S1 = 1 ==> ON

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg2 S1 start published");
      S1_stg_2_timer = millis();
    }

    // Turn OFF S1 when time is over
    if ((S1_state == 1 && (millis() - S1_stg_2_timer >= (N_st2.N_s1_st2_ontime * MINS))) || (MTR_State == 0)) {
      digitalWrite(Q0_3, LOW);  // Output of S1
      S1_state = 0;
      Serial.println("Stage 2 S1 OFF");
      S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg2 S1 stop published");

      S1_stg_2_timer = millis();
    }

    // Calculate the Setpoint every 3 seconds in Function of Ta with the formula : Setpoint = A*(B-Ta)
    if ((millis() - pid_computing_timer >= 3000)) {
      Setpoint = (-(N_SP.N_A * (temp_data.AvgTs_N)) + N_SP.N_B);  //use the average of the temperature over the x last minuites
      setpoint_data.PID_setpoint = float(Setpoint);

      mqtt.publishData(SETPOINT, setpoint_data.PID_setpoint);

      Serial.println("Setpoint published");
      pid_computing_timer = millis();
    }

    // Activate the PID when F1 ON
    if (MTR_State == 1 && (millis() - turn_on_pid_timer >= 3000)) {
      PIDinput = TC1_F;
      coefOutput = (coefPID * Output) / 100;  // Transform the Output of the PID to the desired max value
      Serial.println(coefOutput);
      air_in_feed_PID.Compute();
      // analogWrite(A0_5, Output);
      ledcWrite(AIR_PWM, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      Serial.println("Converted_Output is " + String(Converted_Output));
      turn_on_pid_timer = millis();
    }

    // Put the PID at 0 when F1 OFF
    if (MTR_State == 0 && (millis() - turn_on_pid_timer >= 3000)) {
      //Setpoint = 0;
      PIDinput = 0;
      Output = 0;
      coefOutput = 0;
      // analogWrite(A0_5, Output);
      ledcWrite(AIR_PWM, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      Serial.println("Converted_Output is " + String(Converted_Output));
      turn_off_pid_timer = millis();
    }
  }

  //---- STAGE 3 ----////////////////////////////////////////////////////////////////////////////
  // Initialisation Stage3 (reset all the other stages to 0)
  if (TC2_F >= N_tset.N_ts_set && TC3_F >= N_tset.N_tc_set && Stage3_started == 0 && Stage2_started == 1) {
    START1 = START2 = Stage2_RTC_set = MTR_State = 0;

    // Turn All Output OFF
    ledcWrite(AIR_PWM, 0);
    // analogWrite(A0_5, 0);
    digitalWrite(Q0_0, LOW);
    digitalWrite(Q0_1, LOW);
    digitalWrite(Q0_2, LOW);
    digitalWrite(Q0_3, LOW);
    digitalWrite(Q0_4, LOW);
    Output = 0;
    coefOutput = 0;

    F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("stage 3 F1 init published ");

    S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("stage 2 S1 init published");

    C2_state = S1_state = 0;  // Put the all the states to 0
    Serial.println("Stage 3 Initiated");
    Stage3_started = 1;
  }

  // Stage 3
  if (Stage3_started == 1 && Stage2_started == 1 && STOP == 0) {
    // State of Stage 3 turned to 1
    if (C3_state == 0) {
      digitalWrite(Q0_2, HIGH);  // Turn ON the LED of Stage 3

      C3_state = 1;
      Serial.println("Stage 3 Started");
      setStage(3);
      Serial.println("Stage 3 Status Send packet ");
      F1_stg_3_timer = millis() - (N_st3.N_f1_st3_offtime * MINS);
    }

    // Turn ON F1 when time is over
    if (MTR_State == 0 && (millis() - F1_stg_3_timer >= (N_st3.N_f1_st3_offtime * MINS))) {
      digitalWrite(Q0_4, HIGH);
      // ledcWrite(AIR_PWM, duty_cycle);
      Serial.println("Stage 3 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stage 3 F1 start published ");
      F1_stg_3_timer = millis();
    }

    // Turn OFF F1 when time is over
    if (MTR_State == 1 && (millis() - F1_stg_3_timer >= (N_st3.N_f1_st3_ontime * MINS))) {
      digitalWrite(Q0_4, LOW);
      // ledcWrite(AIR_PWM, 0);
      Serial.println("Stage 3 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stage 3 F1 stop published ");
      F1_stg_3_timer = millis();
    }

    if (S1_state == 0 && (millis() - S1_stg_3_timer >= (N_st3.N_s1_st3_offtime * MINS))) {
      digitalWrite(Q0_3, HIGH);
      S1_state = 1;
      Serial.println("Stage 3 S1 ON");
      S1_data.M_S1 = 1;

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg3 S1 start published");
      S1_stg_3_timer = millis();
    }

    if (S1_state == 1 && (millis() - S1_stg_3_timer >= (N_st3.N_s1_st3_ontime * MINS))) {
      digitalWrite(Q0_3, LOW);
      S1_state = 0;
      Serial.println("Stage 3 S1 OFF with value of S1 ");
      S1_data.M_S1 = 2;

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg3 S1 stop published");
      S1_stg_3_timer = millis();
    }
  }
}

//// fct Callback ==> RECEIVE MQTT MESSAGES ////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int len) {
  Serial.println("Message arrived [" + String(topic) + "]");

  // Delayed start timing
  if (strcmp(topic, sub_hours) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_hour = atof((char *)payload);
    Serial.println("Stage 2 Hours set to: " + String(Stage2_minute));
  }

  if (strcmp(topic, sub_minutes) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_minute = atof((char *)payload);
    Serial.println("Stage 2 Minutes set to: " + String(Stage2_minute));
  }

  if (strcmp(topic, sub_day) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_day = atof((char *)payload);
    Serial.println("Stage 2 Day set to: " + String(Stage2_day));
  }

  if (strcmp(topic, sub_month) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_month = atof((char *)payload);
    Serial.println("Stage 2 Month set to: " + String(N_rtc.N_month));
  }

  //F1 stg1 on/off time
  if (strcmp(topic, sub_f1_st1_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st1.N_f1_st1_ontime = atof((char *)payload);
    Serial.println("F1 Stage 1 on time set to: " + String(N_st1.N_f1_st1_ontime) + " MINS");
  }

  if (strcmp(topic, sub_f1_st1_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st1.N_f1_st1_offtime = atof((char *)payload);
    Serial.println("F1 Stage 1 off time set to: " + String(N_st1.N_f1_st1_offtime) + " MINS");
  }

  // F1 and S1 STAGE 2 on/off time
  if (strcmp(topic, sub_f1_st2_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_f1_st2_ontime = atof((char *)payload);
    Serial.println("F1 Stage 2 on time set to: " + String(N_st2.N_f1_st2_ontime) + " MINS");
  }

  if (strcmp(topic, sub_f1_st2_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_f1_st2_offtime = atof((char *)payload);
    Serial.println("F1 Stage 2 off time set to: " + String(N_st2.N_f1_st2_offtime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st2_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_s1_st2_ontime = atof((char *)payload);
    Serial.println("S1 Stage 2 on time set to: " + String(N_st2.N_s1_st2_ontime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st2_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_s1_st2_offtime = atof((char *)payload);
    Serial.println("S1 Stage 2 off time set to: " + String(N_st2.N_s1_st2_offtime) + " MINS");
  }

  // F1 and S1 STAGE 3 on/off time
  if (strcmp(topic, sub_f1_st3_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_f1_st3_ontime = atof((char *)payload);
    Serial.println("F1 Stage 3 on time set to: " + String(N_st3.N_f1_st3_ontime) + " MINS");
  }

  if (strcmp(topic, sub_f1_st3_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_f1_st3_offtime = atof((char *)payload);
    Serial.println("F1 Stage 3 off time set to: " + String(N_st3.N_f1_st3_offtime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st3_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_s1_st3_ontime = atof((char *)payload);
    Serial.println("S1 Stage 3 on time set to: " + String(N_st3.N_s1_st3_ontime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st3_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_s1_st3_offtime = atof((char *)payload);
    Serial.println("S1 Stage 3 off time set to: " + String(N_st3.N_s1_st3_offtime) + " MINS");
  }

  // Sub A and Sub B value update
  if (strcmp(topic, sub_A) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_SP.N_A = atof((char *)payload);
    Serial.println("A set to: " + String(N_SP.N_A));
    R_A = 1;
  }

  if (strcmp(topic, sub_B) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_SP.N_B = atof((char *)payload);
    Serial.println("B set to: " + String(N_SP.N_B));
    R_B = 1;
  }

  // PID update
  if (strcmp(topic, sub_P) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Kp = atof((char *)payload);
    Serial.println("P set to: " + String(Kp));
    R_P = 1;
  }

  if (strcmp(topic, sub_I) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Ki = atof((char *)payload);
    Serial.println("I set to: " + String(Ki));
    R_I = 1;
  }

  if (strcmp(topic, sub_D) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Kd = atof((char *)payload);
    Serial.println("D set to: " + String(Kd));
    R_D = 1;
  }

  if (R_P == 1 && R_I == 1 && R_D == 1 && START1 == 0 && START2 == 0 && STOP == 0) {
    air_in_feed_PID.SetTunings(Kp, Ki, Kd);
    Serial.println("New PID parameter updated");
    R_P = R_I = R_D = 0;
  }

  if (strcmp(topic, sub_coefPID) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    coefPID = atoi((char *)payload);
    Serial.print("coef PID : " + String(coefPID));
  }

  // Target temperature Ts & Tc update
  if (strcmp(topic, sub_ts_set) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_tset.N_ts_set = atof((char *)payload);
    Serial.println("Ts Condition set to: " + String(N_tset.N_ts_set));
  }

  if (strcmp(topic, sub_tc_set) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_tset.N_tc_set = atof((char *)payload);
    // Tc_cond = N_tset->N_tc_set;
    Serial.println("Tc Condition set to: " + String(N_tset.N_tc_set));
  }

  // START
  if (strcmp(topic, sub_start) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_start = atoi((char *)payload);
    Serial.println("START BUTTON PRESSED ON NODE RED" + String(N_start));
  }

  // D_START
  if ((strcmp(topic, sub_d_start) == 0) && START2 == 0 && STOP == 0) {
    N_d_start = atoi((char *)payload);
    Serial.println("d_start BUTTON PRESSED ON NODE RED" + String(N_d_start));
  }

  // STOP
  if (strcmp(topic, sub_stop) == 0) {
    N_stop = atoi((char *)payload);
    Serial.println("stop BUTTON PRESSED ON NODE RED" + String(N_stop));
  }

  // Choose TS
  if (strcmp(topic, sub_chooseTs) == 0) {
    N_chooseTs = atoi((char *)payload);
    Serial.println("Ts is now IR" + String(N_chooseTs));
  }

  // Address MQTT
  // if (strcmp(topic, sub_address1) == 0) {
  //   Serial.print("Me la pelas");
  //   String S_address1;
  //   char *tmp;
  //   int i = 0;

  //   for (int i = 0; i < len; i++) {
  //     S_address1 += (char)payload[i];
  //   }

  //   tmp = strtok(&S_address1[0], ",");

  //   while (tmp) {
  //     N_address1[i++] = atoi(tmp);
  //     tmp = strtok(NULL, ",");
  //   }

  //   for (int i = 0; i < 8; i++) {
  //     EEPROM.write(i, N_address1[i]);
  //     Serial.println(EEPROM.read(i));
  //   }
  // }

  // if (strcmp(topic, sub_address2) == 0) {
  //   Serial.println(" Test add2");
  //   String S_address2;
  //   char *tmp;
  //   int i = 0;

  //   for (int i = 0; i < len; i++) {
  //     S_address2 += (char)payload[i];
  //   }

  //   tmp = strtok(&S_address2[0], ",");

  //   while (tmp) {
  //     N_address2[i++] = atoi(tmp);
  //     tmp = strtok(NULL, ",");
  //   }

  //   for (int i = 8; i < 16; i++) {
  //     EEPROM.write(i, N_address2[i - 8]);
  //     Serial.println(EEPROM.read(i));
  //   }
  // }

  // if (strcmp(topic, sub_address3) == 0) {
  //   String S_address3;
  //   char *tmp;
  //   int i = 0;

  //   for (int i = 0; i < len; i++) {
  //     S_address3 += (char)payload[i];
  //   }

  //   tmp = strtok(&S_address3[0], ",");

  //   while (tmp) {
  //     N_address3[i++] = atoi(tmp);
  //     tmp = strtok(NULL, ",");
  //   }

  //   for (int i = 16; i < 24; i++) {
  //     EEPROM.write(i, N_address3[i - 16]);
  //     Serial.println(EEPROM.read(i));
  //   }
  //}

  // if (strcmp(topic, sub_address4) == 0) {
  //   String S_address4;
  //   char *tmp;
  //   int i = 0;

  //   for (int i = 0; i < len; i++) {
  //     S_address4 += (char)payload[i];
  //   }

  //   tmp = strtok(&S_address4[0], ",");

  //   while (tmp) {
  //     N_address4[i++] = atoi(tmp);
  //     tmp = strtok(NULL, ",");
  //   }

  //   for (int i = 24; i < 32; i++) {
  //     EEPROM.write(i, N_address4[i - 24]);
  //     Serial.println(EEPROM.read(i));
  //   }
  // }
}

//// Stop button pressed ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stopRoutine() {
  if (stop_temp1 == 0) {
    Serial.println("PROCESS STOP INITIATED");
    digitalWrite(Q0_0, LOW);
    digitalWrite(Q0_1, LOW);
    digitalWrite(Q0_2, LOW);
    digitalWrite(Q0_3, LOW);
    digitalWrite(Q0_4, LOW);
    ledcWrite(AIR_PWM, 0);
    // analogWrite(A0_5, 0);

    stage = 0;
    Output = 0;
    coefOutput = 0;
    stop_temp1 = 1;

    F1_data.M_F1 = S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    mqtt.publishData(m_S1, S1_data.M_S1);
    setStage(0);
    Serial.println("Stage 0 Status Send packet ");
  }

  if (stop_temp2 == 0) {
    MTR_State = C1_state = C2_state = C3_state = S1_state = START1 = START2 = Stage2_started = Stage3_started = Stage2_RTC_set = 0;
    stop_temp2 = 1;
  }

  if (stop_temp2 == 1) {
    Serial.println("PROCESS STOPPED");
    stop_temp1 = stop_temp2 = STOP = 0;
  }
}

void setUpRTC() {
  timeClient.begin();
  timeClient.setTimeOffset(3600 * TIME_ZONE_OFFSET_HRS);
  timeClient.setUpdateInterval(SECS_IN_HR);
  timeClient.update();
  while (!timeClient.updated()) {
    timeClient.update();
    delay(500);
  }
  RTC_DateStruct.Year = timeClient.getYear();
  RTC_DateStruct.Month = timeClient.getMonth();
  RTC_DateStruct.Date = timeClient.getDay();
  M5.Rtc.SetDate(&RTC_DateStruct);
  RTC_TimeStruct.Hours = timeClient.getHours();
  RTC_TimeStruct.Minutes = timeClient.getMinutes();
  RTC_TimeStruct.Seconds = timeClient.getSeconds();
  M5.Rtc.SetTime(&RTC_TimeStruct);
}

void updateTemperature() {
  M5.Rtc.GetTime(&RTC_TimeStruct);
  M5.Rtc.GetDate(&RTC_DateStruct);

  sensors1.requestTemperatures();
  sensors1.requestTemperatures();

  TC1 = sensors1.getTempC(ADDRESS_TC1);  //PV /Ta
  // TC2 = N_chooseTs ? getIRTemp() : sensors1.getTempC(ADDRESS_TC2);// Ts  // Condition to choose if Ts is a IR sensor or OneWire sensor
  TC2 = true ? getIRTemp() : sensors1.getTempC(ADDRESS_TC2);  // Ts  // Condition to choose if Ts is a IR sensor or OneWire sensor
  TC3 = sensors1.getTempC(ADDRESS_TC3);                       // Tc
  TC4 = sensors1.getTempC(ADDRESS_TC4);                       // Ti

  screen.updateTemperatures(TC1, TC2, TC3, TC4);
}

String addressToString(uint8_t *address) {
  String formated_address;
  for (int i = 0; i < 8; i++) {
    formated_address += address[i];
    if (i < 7) formated_address += ",";
  }
  return formated_address;
}

void setStage(int Stage) {
  // if (stage_data.stage == Stage) return;
  stage_data.stage = Stage;
  screen.updateStage(Stage);
  mqtt.publishData(STAGE, Stage);
}

float getIRTemp() {
  uint16_t result;
  float temperature;
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(READ_TEMPERATURE);
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDRESS, 2);
  result = Wire.read();
  result |= Wire.read() << 8;

  temperature = result * 0.02 - 273.15;
  return temperature;
}
