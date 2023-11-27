/* 
################### EDGEBOX Instructions ###################
### https://github.com/espressif/arduino-esp32/pull/7771 ###
############################################################
*/

#include "Thawing-room-EDGEBOX.h"

data_rtc N_rtc;  // structure data_rtc from the config file is renamed N_rtc
data_st1 N_st1;  // fan (zF1) STAGE 1 on and off time
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

// ########################### Timers ##########################
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

// ######################## Temperature ########################
float TA = 0, TA_F = 0;  //Ta
float TS = 0, TS_F = 0;  //Ts
float TC = 0, TC_F = 0;  //Tc
float TI = 0, TI_F = 0;  //Ti optional

// ########################### Buffer ##########################
float avg_ts = 0.0;              // average surface temperature
float buffer_sum = 0;            // variable to store the buffer_sum of the received values
float buffer[BUFFER_SIZE] = {};  // buffer to store the values
uint8_t buffer_len = 0;
uint8_t buffer_index = 0;  // buffer index

MqttClient mqtt;
Controller controller;

PID air_in_feed_PID(&PIDinput, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // DIRECT or REVERSE


void setup() {
  controller.init();

  char SSID[SSID_SIZE];
  char PASS[PASSWORD_SIZE];
  char HOST_NAME[HOSTNAME_SIZE];
  char IP_ADDRESS[IP_ADDRESS_SIZE];
  uint16_t PORT;
  char USERNAME[MQTT_USERNAME_SIZE];
  char PREFIX_TOPIC[MQTT_USERNAME_SIZE];

  runConfigFile(SSID, PASS, HOST_NAME, IP_ADDRESS, &PORT, USERNAME, PREFIX_TOPIC);
  setUpDefaultParameters();

  setStage(0);

  controller.setUpWiFi(SSID, PASS,HOST_NAME);
  controller.connectToWiFi(/* web_server */ true, /* web_serial */ true, /* OTA */ true);
  controller.setUpRTC();

  mqtt.connect(IP_ADDRESS, PORT, USERNAME);
  mqtt.setCallback(callback);
  // mqtt.exampleCall();

  //Turn the PID on
  air_in_feed_PID.SetMode(AUTOMATIC);
  air_in_feed_PID.SetSampleTime(3000);
  //Adjust PID values
  air_in_feed_PID.SetTunings(Kp, Ki, Kd);

  delay(750);
}

void loop() {
  // if is for testing porpuse comment this "if" and replace DateTime "now" for: DateTime now(__DATE__, __TIME__); 
  if (!controller.isRTCConnected()) {  
    logger.println("RTC not connected"); 
    while (true) delay(1000);
  }

  DateTime now = controller.getDateTime();

  if (!controller.isWiFiConnected() && mqtt.isServiceAvailable()) {
    controller.reconnectWiFi();
    delay(500);
    return;
  }

  controller.loopOTA();
  
  if (mqtt.isServiceAvailable()) mqtt.loop();

  updateTemperature();

  // if (N_chooseTs == 1) TC2 = analogRead(A0);                                // Condition to choose if Ts is a IR sensor or OneWire sensor

  if ((TA) > (LOW_TEMP_LIMIT) && (TA) < (HIGH_TEMP_LIMIT)) TA_F = TA;  // if the temperature over the limit it will not be considered

  if ((TS) > (LOW_TEMP_LIMIT) && (TS) < (HIGH_TEMP_LIMIT)) TS_F = TS;

  if ((TC) > (LOW_TEMP_LIMIT) && (TC) < (HIGH_TEMP_LIMIT)) TC_F = TC;

  if ((TI) > (LOW_TEMP_LIMIT) && (TI) < (HIGH_TEMP_LIMIT)) TI_F = TI;

  if (hasIntervalPassed(address_sending_timer,10000U)) {
    
    // String ta_string_address = addressToString(controller.ADDRESS_TA);
    // mqtt.publishData("mduino/sendadd1", ta_string_address);

    // String ts_string_address = addressToString(controller.ADDRESS_TS);
    // mqtt.publishData("mduino/sendadd2", ts_string_address);

    // String tc_string_address = addressToString(controller.ADDRESS_TC);
    // mqtt.publishData("mduino/sendadd3", tc_string_address);

    // String ti_string_address = addressToString(controller.ADDRESS_TI);
    // mqtt.publishData("mduino/sendadd4", ti_string_address);
  }

  //---- Get surface temperature average with a FIFO buffer ---- //////////////////////////////// Something fuckin' wrong with the average
  if (hasIntervalPassed(ts_avg_timer, AVG_RESOLUTION)) {
    // if (millis() - ts_avg_timer >= AVG_RESOLUTION)
    if (buffer_len < BUFFER_SIZE) { //if buffer not full, we add the value
        buffer_sum += TS_F;
        buffer[buffer_len] = TS_F;
        buffer_len++;
      }
      else { //buffer full, we remove the oldest value and add the new one
        buffer_sum -= buffer[buffer_index];
        buffer[buffer_index] = TS_F;
        buffer_sum += TS_F;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE; // update the buffer index
      }
      
      avg_ts = buffer_sum/buffer_len;

    mqtt.publishData(AVG_TS_TOPIC, temp_data.AvgTs_N);
    // logger.println("Temp data published");
  }

  //---- Temperature MQTT publish ----///////////////////////////////////////////////////////////
  if (hasIntervalPassed(get_temp_timer, TIME_ACQ_DELAY)) {
    temp_data.Ta_N = TA_F;
    temp_data.Ts_N = TS_F;
    temp_data.Tc_N = TC_F;
    temp_data.Ti_N = TI_F;
    temp_data.AvgTs_N = avg_ts;

    mqtt.publishData(TA_TOPIC, temp_data.Ta_N);
    mqtt.publishData(TS_TOPIC, temp_data.Ts_N);
    mqtt.publishData(TC_TOPIC, temp_data.Tc_N);
    mqtt.publishData(TI_TOPIC, temp_data.Ti_N);

    // for debug purpose
    logger.println("Average: " + String(temp_data.AvgTs_N));
    logger.println(String(controller.readDigitalInput(DI0)));
    logger.println("Ts: " + String(TS));
    logger.println("TC: " + String(TC));
    logger.println("Ta: " + String(TA));
    logger.println("Nstart: " + String(N_start));
    logger.println("Nstop: " + String(N_stop));
    logger.println("A variable: " + String(N_SP.N_A));
    logger.println("B variable: " + String(N_SP.N_B));
    logger.println("P variable: " + String(Kp));
    logger.println("I variable: " + String(Ki));
    logger.println("D variable: " + String(Kd));
    logger.println("setpoint raw: " + String(Setpoint));
    logger.println("setpoint: " + String(setpoint_data.PID_setpoint));

    logger.printTime("Time:", now.hour(), now.minute(), now.day(), now.month());
    logger.printTime("Stage 2 Time:", Stage2_hour, Stage2_minute, Stage2_day, Stage2_month);
  }
  
  //---- Time Stage ON/OFF and A & B MQTT Publish ----///////////////////////////////////////////////////////
  if (hasIntervalPassed(A_B_timer, 10000)) {
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
  }

  //---- PID Publishing ----//////////////////////////////////////////////////////////////////////
  // PID works only on STAGE 2
  if (stage == 2 && !STOP ) {
    if (hasIntervalPassed(stg_2_pid_timer, TIME_ACQ_DELAY + 1)) {
      logger.println("Soft PID Actual Output is" + String(Output));
      Output_float = float(coefOutput);
      PID_data.PID_output = ((Output_float - 0) / (255 - 0)) * (100 - 0) + 0;
      logger.println("PID Output /100 is" + String(PID_data.PID_output));

      mqtt.publishData(PID_OUTPUT, PID_data.PID_output);
    }
  }

  //---- START, DELAYED, STOP Button pressed ----////////////////////////////////////////////////
  // delayed start push button or digital button pressed
  if (controller.readDigitalInput(DLY_S_IO) || N_d_start == 1) {
    START1 = true;
    logger.println("Delayed Start Pressed");
    N_d_start = 0;
    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    logger.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    logger.println("Stage 1 init M_S1 stop published");
  }

  // start push button or digital button pressed
  if (controller.readDigitalInput(START_IO) || N_start == 1) {
    START2 = true;
    logger.println("Start Pressed");
    N_start = 0;
    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    logger.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    logger.println("Stage 1 init M_S1 stop published");
  }

  // stop push button or digital button pressed
  if (controller.readDigitalInput(STOP_IO) || N_stop == 1) {
    STOP = true;
    logger.println("Stop Pressed");
    N_stop = 0;
  }

  //---- STOP ROUTINE ----///////////////////////////////////////////////////////////////////////
  if (STOP) stopRoutine();
  //---- RTC Timer ----//////////////////////////////////////////////////////////////////////////

  bool is_after_stage2_time = now.hour() >= Stage2_hour && now.minute() >= Stage2_minute;
  bool is_after_stage_2_date = now.day() >= Stage2_day && now.month() >= Stage2_month;

  bool is_after_stage_2_start = is_after_stage2_time && is_after_stage_2_date;

  bool is_stage_2_triggered = (is_after_stage_2_start && START1) || START2;
  bool is_stage_2_ready = !Stage2_started && !Stage2_RTC_set;

  if (is_stage_2_triggered && is_stage_2_ready)  {

    START1 = MTR_State = C1_state = 0;
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);

    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    logger.println("All M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    logger.println("All M_S1 stop published");

    logger.println("Stage 2 Initiated wait for 5 secs");
    Stage2_RTC_set = Stage2_started = 1;
    delay(5000);
  }

  //---- STAGE 1 ----////////////////////////////////////////////////////////////////////////////
  if (START1 && !Stage2_RTC_set && !STOP) {
    if (!C1_state) {
      controller.writeDigitalOutput(STAGE_1_IO, HIGH);  // Turn On the LED of Stage 1
      C1_state = 1;                    // State of Stage 1 turned ON
      logger.println("Stage 1 Started");
      setStage(1);
      logger.println("Stage 1 Status Send packet ");
      F1_timer = millis() - (N_st1.N_f1_st1_ontime * MINS);
    }

    // Turn ON F1
    if (!MTR_State && !controller.readDigitalInput(FAN_IO) && hasIntervalPassed(F1_timer, N_st1.N_f1_st1_offtime , true)) {
      controller.writeDigitalOutput(FAN_IO, HIGH);                                                                                       // Turn ON F1
      logger.println("Stage 1 F1 On");
      MTR_State = true;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      logger.println("Stage 1 init M_F1 ON published ");
    }

    // Turn OFF F1 when the time set in the configuration is over
    if (MTR_State && controller.readDigitalInput(FAN_IO) && hasIntervalPassed(F1_timer, N_st1.N_f1_st1_ontime , true)) {
      controller.writeDigitalOutput(FAN_IO, LOW);
      // controller.writeAnalogOutput(AIR_PWM, 0);
      logger.println("Stage 1 F1 Off");
      MTR_State = false;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      logger.println("Stage 1 init M_F1 OFF published ");
    }
  }

  //---- STAGE 2 ----////////////////////////////////////////////////////////////////////////////
  if (Stage2_RTC_set && !Stage3_started && !STOP) {
    if (!C2_state) {
      controller.writeDigitalOutput(STAGE_2_IO, HIGH);  // Turn On the LED of Stage 2

      C2_state = true;
      logger.println("Stage 2 Started");
      stage = 2;
      setStage(2);
      logger.println("Stage 0 Status Send packet ");
      F1_stg_2_timmer = millis() - (N_st2.N_f1_st2_offtime * MINS);
    }

    // Turn ON F1 when time is over
    if (!MTR_State && hasIntervalPassed(F1_stg_2_timmer, N_st2.N_f1_st2_offtime, true)) {
      controller.writeDigitalOutput(FAN_IO, HIGH);  // Output of F1
      logger.println("Stage 2 F1 On");
      MTR_State = true;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      logger.println("stg2 F1 Start published ");
    }

    // Turn OFF F1 when time is over
    if (MTR_State && hasIntervalPassed(F1_stg_2_timmer, N_st2.N_f1_st2_ontime, true) ){
      controller.writeDigitalOutput(FAN_IO, LOW);
      logger.println("Stage 2 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      logger.println("stg2 F1 stop published ");
    }

    // Turn ON S1 when time is over
    if (MTR_State && !S1_state && hasIntervalPassed(S1_stg_2_timer,N_st2.N_s1_st2_offtime, true)) {    
      controller.writeDigitalOutput(VALVE_IO, HIGH);  // Output of S1
      S1_state = 1;
      logger.println("Stage 2 S1 ON");
      S1_data.M_S1 = 1;  // When M_S1 = 1 ==> ON

      mqtt.publishData(m_S1, S1_data.M_S1);
      logger.println("stg2 S1 start published");
    }

    // Turn OFF S1 when time is over
    if ((S1_state && hasIntervalPassed(S1_stg_2_timer,N_st2.N_s1_st2_ontime, true)) || !MTR_State ) {    
      controller.writeDigitalOutput(VALVE_IO, LOW);  // Output of S1
      S1_state = 0;
      logger.println("Stage 2 S1 OFF");
      S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

      mqtt.publishData(m_S1, S1_data.M_S1);
      logger.println("stg2 S1 stop published");
    }

    // Calculate the Setpoint every 3 seconds in Function of Ta with the formula : Setpoint = A*(B-Ta)
    if (hasIntervalPassed(pid_computing_timer, 3000)) {
      Setpoint = (-(N_SP.N_A * (temp_data.AvgTs_N)) + N_SP.N_B);  //use the average of the temperature over the x last minuites
      setpoint_data.PID_setpoint = float(Setpoint);

      mqtt.publishData(SETPOINT, setpoint_data.PID_setpoint);

      logger.println("Setpoint published");
    }

    // Activate the PID when F1 ON
    if (MTR_State && hasIntervalPassed(turn_on_pid_timer, 3000)) {
      PIDinput = TA_F;
      coefOutput = (coefPID * Output) / 100;  // Transform the Output of the PID to the desired max value
      logger.println(String(coefOutput));
      air_in_feed_PID.Compute();
      // analogWrite(A0_5, Output);
      controller.writeAnalogOutput(AIR_PWM, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      logger.println("Converted_Output is " + String(Converted_Output));
    }

    // Put the PID at 0 when F1 OFF
    if (!MTR_State && hasIntervalPassed(turn_on_pid_timer, 3000)) {
      //Setpoint = 0;
      PIDinput = 0;
      Output = 0;
      coefOutput = 0;
      // analogWrite(A0_5, Output);
      controller.writeAnalogOutput(AIR_PWM, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      logger.println("Converted_Output is " + String(Converted_Output));
    }
  }

  //---- STAGE 3 ----////////////////////////////////////////////////////////////////////////////
  // Initialisation Stage3 (reset all the other stages to 0)
  if (TS_F >= N_tset.N_ts_set && TC_F >= N_tset.N_tc_set && !Stage3_started && Stage2_started) {
    START1 = START2 = Stage2_RTC_set = MTR_State = false;

    // Turn All Output OFF
    controller.writeAnalogOutput(AIR_PWM, 0);
    // analogWrite(A0_5, 0);
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);
    Output = 0;
    coefOutput = 0;

    F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

    mqtt.publishData(m_F1, F1_data.M_F1);
    logger.println("stage 3 F1 init published ");

    S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

    mqtt.publishData(m_S1, S1_data.M_S1);
    logger.println("stage 2 S1 init published");

    C2_state = S1_state = 0;  // Put the all the states to 0
    logger.println("Stage 3 Initiated");
    Stage3_started = 1;
  }

  // Stage 3
  if (Stage3_started && Stage2_started && !STOP) {
    // State of Stage 3 turned to 1
    if (!C3_state) {
      controller.writeDigitalOutput(STAGE_3_IO, HIGH);  // Turn ON the LED of Stage 3

      C3_state = 1;
      logger.println("Stage 3 Started");
      setStage(3);
      logger.println("Stage 3 Status Send packet ");
      F1_stg_3_timer = millis() - (N_st3.N_f1_st3_offtime * MINS);
    }

    // Turn ON F1 when time is over
    if(MTR_State && hasIntervalPassed(F1_stg_3_timer, N_st3.N_f1_st3_offtime, true)) {
      controller.writeDigitalOutput(FAN_IO, HIGH);
      // controller.writeAnalogOutput(AIR_PWM, duty_cycle);
      logger.println("Stage 3 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;

      mqtt.publishData(m_F1, F1_data.M_F1);
      logger.println("stage 3 F1 start published ");
    }

    // Turn OFF F1 when time is over
    if(MTR_State && hasIntervalPassed(F1_stg_3_timer, N_st3.N_f1_st3_ontime, true)) {
      controller.writeDigitalOutput(FAN_IO, LOW);
      // controller.writeAnalogOutput(AIR_PWM, 0);
      logger.println("Stage 3 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;

      mqtt.publishData(m_F1, F1_data.M_F1);
      logger.println("stage 3 F1 stop published ");
    }

    if (!S1_state && hasIntervalPassed(S1_stg_3_timer, N_st3.N_s1_st3_offtime, true)) {
      controller.writeDigitalOutput(VALVE_IO, HIGH);
      S1_state = 1;
      logger.println("Stage 3 S1 ON");
      S1_data.M_S1 = 1;

      mqtt.publishData(m_S1, S1_data.M_S1);
      logger.println("stg3 S1 start published");
    }

    if (S1_state && hasIntervalPassed(S1_stg_3_timer, N_st3.N_s1_st3_ontime, true)) {
      controller.writeDigitalOutput(VALVE_IO, LOW);
      S1_state = 0;
      logger.println("Stage 3 S1 OFF with value of S1 ");
      S1_data.M_S1 = 2;

      mqtt.publishData(m_S1, S1_data.M_S1);
      logger.println("stg3 S1 stop published");
    }
  }
}

//// fct Callback ==> RECEIVE MQTT MESSAGES ////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int len) {
  logger.println("Message arrived [" + String(topic) + "]");

  // Delayed start timing
  if (mqtt.isTopicEqual(topic, sub_hours) && noButtonPressed()) {
    Stage2_hour = responseToFloat(payload, len);
    logger.println("Stage 2 Hours set to: " + String(Stage2_minute));
  }

  if (mqtt.isTopicEqual(topic, sub_minutes) && noButtonPressed()) {
    Stage2_minute = responseToFloat(payload, len);
    logger.println("Stage 2 Minutes set to: " + String(Stage2_minute));
  }

  if (mqtt.isTopicEqual(topic, sub_day) && noButtonPressed()) {
    Stage2_day = responseToFloat(payload, len);
    logger.println("Stage 2 Day set to: " + String(Stage2_day));
  }

  if (mqtt.isTopicEqual(topic, sub_month) && noButtonPressed()) {
    Stage2_month = responseToFloat(payload, len);
    logger.println("Stage 2 Month set to: " + String(N_rtc.N_month));
  }

  bool update_default_parameters = false;

  //F1 stg1 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st1_ontime) && noButtonPressed()) {
    N_st1.N_f1_st1_ontime = responseToFloat(payload, len);
    logger.println("F1 Stage 1 on time set to: " + String(N_st1.N_f1_st1_ontime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st1_offtime) && noButtonPressed()) {
    N_st1.N_f1_st1_offtime = responseToFloat(payload, len);
    logger.println("F1 Stage 1 off time set to: " + String(N_st1.N_f1_st1_offtime) + " MINS");
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 2 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st2_ontime) && noButtonPressed()) {
    N_st2.N_f1_st2_ontime = responseToFloat(payload, len);
    logger.println("F1 Stage 2 on time set to: " + String(N_st2.N_f1_st2_ontime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st2_offtime) && noButtonPressed()) {
    N_st2.N_f1_st2_offtime = responseToFloat(payload, len);
    logger.println("F1 Stage 2 off time set to: " + String(N_st2.N_f1_st2_offtime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st2_ontime) && noButtonPressed()) {
    N_st2.N_s1_st2_ontime = responseToFloat(payload, len);
    logger.println("S1 Stage 2 on time set to: " + String(N_st2.N_s1_st2_ontime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st2_offtime) && noButtonPressed()) {
    N_st2.N_s1_st2_offtime = responseToFloat(payload, len);
    logger.println("S1 Stage 2 off time set to: " + String(N_st2.N_s1_st2_offtime) + " MINS");
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 3 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st3_ontime) && noButtonPressed()) {
    N_st3.N_f1_st3_ontime = responseToFloat(payload, len);
    logger.println("F1 Stage 3 on time set to: " + String(N_st3.N_f1_st3_ontime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st3_offtime) && noButtonPressed()) {
    N_st3.N_f1_st3_offtime = responseToFloat(payload, len);
    logger.println("F1 Stage 3 off time set to: " + String(N_st3.N_f1_st3_offtime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st3_ontime) && noButtonPressed()) {
    N_st3.N_s1_st3_ontime = responseToFloat(payload, len);
    logger.println("S1 Stage 3 on time set to: " + String(N_st3.N_s1_st3_ontime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st3_offtime) && noButtonPressed()) {
    N_st3.N_s1_st3_offtime = responseToFloat(payload, len);
    logger.println("S1 Stage 3 off time set to: " + String(N_st3.N_s1_st3_offtime) + " MINS");
    update_default_parameters = true;
  }

  // Sub A and Sub B value update
  if (mqtt.isTopicEqual(topic, sub_A) && noButtonPressed()) {
    N_SP.N_A = responseToFloat(payload, len);
    // N_SP.N_A = atoi((char *)payload);
    logger.println("A set to: " + String(N_SP.N_A));
    update_default_parameters = true;
    R_A = 1;
  }

  if (mqtt.isTopicEqual(topic, sub_B) && noButtonPressed()) {
    N_SP.N_B = responseToFloat(payload, len);
    logger.println("B set to: " + String(N_SP.N_B));
    update_default_parameters = true;
    R_B = 1;
  }

  // PID update
  if (mqtt.isTopicEqual(topic, sub_P) && noButtonPressed()) {
    Kp = responseToFloat(payload, len);
    logger.println("P set to: " + String(Kp));
    R_P = 1;
  }

  if (mqtt.isTopicEqual(topic, sub_I) && noButtonPressed()) {
    Ki = responseToFloat(payload, len);
    logger.println("I set to: " + String(Ki));
    R_I = 1;
  }

  if (mqtt.isTopicEqual(topic, sub_D) && noButtonPressed()) {
    Kd = responseToFloat(payload, len);
    logger.println("D set to: " + String(Kd));
    R_D = 1;
  }

  if (R_P == 1 && R_I == 1 && R_D == 1 && noButtonPressed()) {
    air_in_feed_PID.SetTunings(Kp, Ki, Kd);
    logger.println("New PID parameter updated");
    R_P = R_I = R_D = 0;
  }

  if (mqtt.isTopicEqual(topic, sub_coefPID) && noButtonPressed()) {
    coefPID = responseToInt(payload, len);
    logger.print("coef PID : " + String(coefPID));
  }

  // Target temperature Ts & Tc update
  if (mqtt.isTopicEqual(topic, sub_ts_set) && noButtonPressed()) {
    N_tset.N_ts_set = responseToFloat(payload, len);
    logger.println("Ts Condition set to: " + String(N_tset.N_ts_set));
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_tc_set) && noButtonPressed()) {
    N_tset.N_tc_set = responseToFloat(payload, len);
    // Tc_cond = N_tset->N_tc_set;
    logger.println("Tc Condition set to: " + String(N_tset.N_tc_set));
    update_default_parameters = true;
  }

  // START
  if (mqtt.isTopicEqual(topic, sub_start) && noButtonPressed()) {
    N_start = responseToInt(payload, len);
    logger.println("START BUTTON PRESSED ON NODE RED" + String(N_start));
  }

  // D_START
  if (mqtt.isTopicEqual(topic, sub_d_start) && START2 == 0 && STOP == 0) {
    N_d_start = responseToInt(payload, len);
    logger.println("d_start BUTTON PRESSED ON NODE RED" + String(N_d_start));
  }

  // STOP
  if (mqtt.isTopicEqual(topic, sub_stop)) {
    N_stop = responseToInt(payload, len);
    logger.println("stop BUTTON PRESSED ON NODE RED" + String(N_stop));
  }

  // Choose TS
  if (mqtt.isTopicEqual(topic, sub_chooseTs)) {
    N_chooseTs = responseToInt(payload, len);
    logger.println("Ts is now IR" + String(N_chooseTs));
  }

  if(update_default_parameters) updateDefaultParameters();
}

void updateDefaultParameters(){
  // Abre el archivo de configuración existente
  File configFile = SPIFFS.open("/defaultParameters.txt", FILE_READ);
  if (!configFile) {
    Serial.println("Error al abrir el archivo de configuración para lectura");
    return;
  }

  // Lee el contenido en una cadena
  String content = configFile.readString();
  configFile.close();

  // Parsea el objeto JSON del archivo
  StaticJsonDocument<1024> doc; // Cambiado a StaticJsonDocument
  auto error = deserializeJson(doc, content);
  if (error) {
    Serial.println("Error al parsear el archivo de configuración");
    return;
  }

  // Update the values
  doc["stage1"]["f1Ontime"] = N_st1.N_f1_st1_ontime;
  doc["stage1"]["f1Offtime"] = N_st1.N_f1_st1_offtime;

  doc["stage2"]["f1Ontime"] = N_st2.N_f1_st2_ontime;
  doc["stage2"]["f1Offtime"] = N_st2.N_f1_st2_offtime;
  doc["stage2"]["s1Ontime"] = N_st2.N_s1_st2_ontime;
  doc["stage2"]["s1Offtime"] = N_st2.N_s1_st2_offtime;

  doc["stage3"]["f1Ontime"] = N_st3.N_f1_st3_ontime;
  doc["stage3"]["f1Offtime"] = N_st3.N_f1_st3_offtime;
  doc["stage3"]["s1Ontime"] = N_st3.N_s1_st3_ontime;
  doc["stage3"]["s1Offtime"] = N_st3.N_s1_st3_offtime;

  doc["setPoint"]["A"] = N_SP.N_A;
  doc["setPoint"]["B"] = N_SP.N_B;
  
  doc["tset"]["tsSet"] = N_tset.N_ts_set;
  doc["tset"]["tcSet"] = N_tset.N_tc_set;

  // Open file for writing
  configFile = SPIFFS.open("/defaultParameters.txt", FILE_WRITE);
  if (!configFile) {
    Serial.println("Error al abrir el archivo de configuración para escritura");
    return;
  }

  // Serializa el JSON al archivo
  if (serializeJson(doc, configFile) == 0) {
    Serial.println("Error al escribir en el archivo de configuración");
  }

  configFile.close();
}

//// Stop button pressed ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stopRoutine() {
  if (!stop_temp1) {
    logger.println("PROCESS STOP INITIATED");
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);
    controller.writeAnalogOutput(AIR_PWM, 0);
    // analogWrite(A0_5, 0);

    stage = 0;
    Output = 0;
    coefOutput = 0;
    stop_temp1 = true;

    F1_data.M_F1 = S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    mqtt.publishData(m_S1, S1_data.M_S1);
    setStage(0);
    logger.println("Stage 0 Status Send packet ");
  }

  if (!stop_temp2) {
    MTR_State = C1_state = C2_state = C3_state = S1_state = START1 = START2 = Stage2_started = Stage3_started = Stage2_RTC_set = false;
    stop_temp2 = true;
  }

  if (stop_temp2) {
    logger.println("PROCESS STOPPED");
    stop_temp1 = stop_temp2 = STOP = false;
  }
}

void updateTemperature() {
  controller.updateProbesTemperatures();

  const float ta_raw = controller.readTempFrom(TA_AI);  // Ta
  const float ts_raw = controller.readTempFrom(TS_AI);  // Ts
  const float tc_raw = controller.readTempFrom(TC_AI);  // Tc

  TA = validateTemperature(ta_raw, TA_TYPE) ? ta_raw : TA_DEF;
  TS = validateTemperature(ts_raw, TS_TYPE) ? ts_raw : TS_DEF;
  TC = validateTemperature(tc_raw, TC_TYPE) ? tc_raw : TC_DEF;

  TI = controller.getOneWireTempFrom(controller.ADDRESS_TI);  // Ti
}

bool validateTemperature(float temp, uint8_t type) {
  switch (type) {
    case TA_TYPE:
      if (temp >= TA_MIN && temp <= TA_MAX) return true;
      else sendTemperaturaAlert(temp, "TA");
      break;
    case TS_TYPE:
      if (temp >= TS_MIN && temp <= TS_MAX) return true;
      else sendTemperaturaAlert(temp, "TS");
      break;
    case TC_TYPE:
      if (temp >= TC_MIN && temp <= TC_MAX) return true;
      else sendTemperaturaAlert(temp, "TC");
      break;
  }
  return false;
}

// THIS SHOULD BE ALSO IN THE MQTT

void sendTemperaturaAlert(float temp, String sensor){
  const String msg = "{\"temp\":" + String(temp) + ", \"sensor\":" + sensor + "}";
  mqtt.publishData(SPOILED_SENSOR, msg);
}

// THIS SHOULD BE ALSO IN THE CONTROLLER

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
  mqtt.publishData(STAGE, Stage);
}

// THIS SHOULD BE ALSO IN THE OTHER PLACE (MQTT maybe )

float responseToFloat(byte *value, size_t len) {
  String string_builder;
  for (int i = 0; i < len; i++) string_builder += (char)value[i];
  return string_builder.toFloat();
}

// THIS SHOULD BE ALSO IN THE OTHER PLACE

int responseToInt(byte *value, size_t len) {
  String string_builder;
  for (int i = 0; i < len; i++) string_builder += (char)value[i];
  return string_builder.toInt();
}

// float getIRTemp() {
//   uint16_t result;
//   float temperature;
//   Wire.beginTransmission(IR_SENSOR_ADDRESS);
//   Wire.write(READ_TEMPERATURE);
//   Wire.endTransmission(false);
//   Wire.requestFrom(IR_SENSOR_ADDRESS, 2);
//   result = Wire.read();
//   result |= Wire.read() << 8;

//   temperature = result * 0.02 - 273.15;
//   return temperature;
// }

// THIS SHOULD BE ALSO IN THE CONTROLLER

void setUpDefaultParameters(){

  File file = SPIFFS.open("/defaultParameters.txt", "r");
  if (!file) {
    Serial.println("Error al abrir el archivo de parámetros");
    return;
  }

  String jsonText = file.readString();
  file.close();

  // Parsea el JSON
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, jsonText);
  if (error) {
    Serial.println("Error al parsear el JSON");
    return;
  }

  N_st1.N_f1_st1_ontime = doc["stage1"]["f1Ontime"];
  N_st1.N_f1_st1_offtime = doc["stage1"]["f1Offtime"];

  N_st2.N_f1_st2_ontime = doc["stage2"]["f1Ontime"];
  N_st2.N_f1_st2_offtime = doc["stage2"]["f1Offtime"];
  N_st2.N_s1_st2_ontime = doc["stage2"]["s1Ontime"];
  N_st2.N_s1_st2_offtime = doc["stage2"]["s1Offtime"];

  N_st3.N_f1_st3_ontime = doc["stage3"]["f1Ontime"];
  N_st3.N_f1_st3_offtime = doc["stage3"]["f1Offtime"];
  N_st3.N_s1_st3_ontime = doc["stage3"]["s1Ontime"];
  N_st3.N_s1_st3_offtime = doc["stage3"]["s1Offtime"];

  N_SP.N_A = doc["setPoint"]["A"];
  N_SP.N_B = doc["setPoint"]["B"];; 

  N_tset.N_ts_set = doc["tset"]["tsSet"];
  N_tset.N_tc_set = doc["tset"]["tcSet"];

  // Imprime los valores de las variables
  // logger.println("Valores de las variables:");
  // logger.printValue("N_st1.N_f1_st1_ontime: ",  String(N_st1.N_f1_st1_ontime)); 
  // logger.printValue("N_st1.N_f1_st1_offtime: ", String(N_st1.N_f1_st1_offtime));
  
  // logger.printValue("N_st2.N_f1_st2_ontime: ", String(N_st2.N_f1_st2_ontime));
  // logger.printValue("N_st2.N_f1_st2_offtime: ", String(N_st2.N_f1_st2_offtime));
  // logger.printValue("N_st2.N_s1_st2_ontime: ", String(N_st2.N_s1_st2_ontime));
  // logger.printValue("N_st2.N_s1_st2_offtime: ", String(N_st2.N_s1_st2_offtime));

  // logger.printValue("N_st3.N_f1_st3_ontime: ", String(N_st3.N_f1_st3_ontime));
  // logger.printValue("N_st3.N_f1_st3_offtime: ", String(N_st3.N_f1_st3_offtime));
  // logger.printValue("N_st3.N_s1_st3_ontime: ", String(N_st3.N_s1_st3_ontime));
  // logger.printValue("N_st3.N_s1_st3_offtime: ", String(N_st3.N_s1_st3_offtime));

  // logger.printValue("N_SP.N_A: ", String( N_SP.N_A));
  // logger.printValue("N_SP.N_B: ", String( N_SP.N_B));

  // logger.printValue("N_tset.N_ts_set: ", String( N_tset.N_ts_set));
  // logger.printValue("N_tset.N_tc_set: ", String( N_tset.N_tc_set));
}

// THIS SHOULD BE ALSO IN THE OTHER PLACE

void runConfigFile(char* ssid, char* password, char* hostname, char* ip_address, uint16_t* port, char* username, char* prefix_topic) {
  // Iniciar SPIFFS
  if (!SPIFFS.begin(true)) {
    logger.println("An error has occurred while mounting SPIFFS");
    return;
  }

  // Leer archivo de configuración
  File file = SPIFFS.open("/config.txt");
  if (!file) {
    logger.println("Failed to open config file");
    return;
  }

  // Tamaño para el documento JSON
  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);
  file.readBytes(buf.get(), size);
  file.close();

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, buf.get());
  if (error) {
    Serial.println("Failed to parse config file");
    return;
  }

  // Asignar valores y verificar si están presentes en el JSON
  if (doc.containsKey("SSID")) strlcpy(ssid, doc["SSID"], SSID_SIZE);
  if (doc.containsKey("WIFI_PASSWORD")) strlcpy(password, doc["WIFI_PASSWORD"], PASSWORD_SIZE);
  if (doc.containsKey("HOST_NAME")) strlcpy(hostname, doc["HOST_NAME"], HOSTNAME_SIZE);
  if (doc.containsKey("IP_ADDRESS")) strlcpy(ip_address, doc["IP_ADDRESS"], IP_ADDRESS_SIZE);
  if (doc.containsKey("PORT")) *port = doc["PORT"];
  if (doc.containsKey("USERNAME")) strlcpy(username, doc["USERNAME"], MQTT_USERNAME_SIZE);
  if (doc.containsKey("TOPIC")) strlcpy(prefix_topic, doc["TOPIC"], MQTT_USERNAME_SIZE);
}

bool noButtonPressed(){
  return START1 == 0 && START2 == 0 && STOP == 0;
}

bool hasIntervalPassed(uint32_t &previousMillis, uint32_t interval, bool to_min) {
  if(to_min) interval *= 60000;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Restablecer el temporizador después de que ha pasado el intervalo
    return true;
  }
  return false; 
}


