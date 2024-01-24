#include "Thawing-room-EDGEBOX.h"

// Stage parameters
stage_parameters stage1_params;
stage_parameters stage2_params;  // fan (F1) and sprinklers (S1) STAGE 2 on and off time
stage_parameters stage3_params;

// A & B variables
room_parameters room;

// Temperatures measures of Ta, Ts, Tc, Ti & AvgTs
data_s temp_data;

// Ts & Tc target value
data_tset temp_set;

uint8_t fan_1;
uint8_t sprinkler_1;


bool sprinkler_1_state = false;
bool mtr_state = false;  // State of the motor that control the Fan F1


// PID parameters
float pid_output, pid_setpoint;           // value of the PID output
double Kp = 0, Ki = 10, Kd = 0;
double Output, pid_input, Setpoint;

bool R_P = true;
bool R_I = true;
bool R_D = true;

double coef_output = 0;  // Output for the infeed (New Analog Output that will be sent to S1
uint8_t coef_pid = 100;
uint8_t Converted_Output = 0;

bool stop_temp1 = false;
bool stop_temp2 = false;

// Start, delayed start, stop, and choose Ts
bool remote_stop = false;
bool remote_start = false;
bool remote_d_start = false;

uint8_t chooseTs = 0;

bool STOP = false;
bool START = false;
bool START1 = false;     // delayed start bttn
bool START2 = false;     // start bttn

// Parameters of Stage 2
uint8_t stage2_hour = 0;
uint8_t stage2_minute = 0;
uint8_t stage2_day = 0;
uint8_t stage2_month = 0;

bool stage2_rtc_set = 0;
bool stage2_started = 0;
bool stage3_started = 0;

// ########################### Timers ##########################
uint32_t fan_1_timer = 0UL;               // fan F1 timing
uint32_t pid_computing_timer = 0UL;    // PID computing timing
uint32_t fan_1_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t sprinkler_1_stg_2_timer = 0UL;         // S1 stage 2 timing
uint32_t fan_1_stg_3_timer = 0UL;         // F1 stage 3 timing
uint32_t sprinkler_1_stg_3_timer = 0UL;         // S1 stage 3 timing
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

SystemState currentState = IDLE;

MqttClient mqtt;
Controller controller;
TaskHandle_t communicationTask;
PID air_in_feed_PID(&pid_input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // DIRECT or REVERSE

void backgroundTasks(void* pvParameters) {
  for (;;) {
    controller.WiFiLoop();

    if(controller.isWiFiConnected()) {
      mqtt.loop();
      controller.loopOTA();
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  controller.init();

  char SSID[SSID_SIZE];
  char PASS[PASSWORD_SIZE];
  char HOST_NAME[HOSTNAME_SIZE];
  char IP_ADDRESS[IP_ADDRESS_SIZE];
  uint16_t PORT;
  char USERNAME[MQTT_USERNAME_SIZE];
  char PREFIX_TOPIC[MQTT_USERNAME_SIZE];

  controller.runConfigFile(SSID, PASS, HOST_NAME, IP_ADDRESS, &PORT, USERNAME, PREFIX_TOPIC);
  controller.setUpDefaultParameters(stage1_params, stage2_params, stage3_params, room, temp_set);

  setStage(IDLE);

  controller.setUpWiFi(SSID, PASS,HOST_NAME);
  controller.connectToWiFi(true, true, true);
  controller.setUpRTC();

  mqtt.connect(IP_ADDRESS, PORT, USERNAME);
  mqtt.setCallback(callback);
  // mqtt.exampleCall();

  xTaskCreatePinnedToCore(backgroundTasks, "communicationTask", 10000, NULL, 1, &communicationTask, 0);

  //Turn the PID on
  air_in_feed_PID.SetMode(AUTOMATIC);
  air_in_feed_PID.SetSampleTime(3000);
  air_in_feed_PID.SetTunings(Kp, Ki, Kd);

  delay(750);
}

void loop() {
  // if is for testing porpuse comment this "if" and replace DateTime "now" for: DateTime now(__DATE__, __TIME__); 
  DateTime current_date = controller.getDateTime();

  if (!controller.isRTCConnected()) {  
    logger.println("RTC not connected"); 
    while (true) delay(1000);
  }

  updateTemperature();

  if ((TA) > (LOW_TEMP_LIMIT) && (TA) < (HIGH_TEMP_LIMIT)) TA_F = TA;  // if the temperature over the limit it will not be considered
  if ((TS) > (LOW_TEMP_LIMIT) && (TS) < (HIGH_TEMP_LIMIT)) TS_F = TS;
  if ((TC) > (LOW_TEMP_LIMIT) && (TC) < (HIGH_TEMP_LIMIT)) TC_F = TC;
  if ((TI) > (LOW_TEMP_LIMIT) && (TI) < (HIGH_TEMP_LIMIT)) TI_F = TI;

  if (hasIntervalPassed(ts_avg_timer, AVG_RESOLUTION)) getTsAvg();
  if (hasIntervalPassed(get_temp_timer, TIME_ACQ_DELAY)) publishTemperatures(current_date);  
  if (hasIntervalPassed(A_B_timer, 10000)) aknowledgementRoutine();
  if (currentState == STAGE2 && !STOP ) publishPID();  // PID works only on STAGE 2

  const bool start_stage2 = shouldStage2Start(current_date);
  const bool start_stage3 = shouldStage3Start(current_date);

  handleInputs();
  if (STOP) stopRoutine();

  // ---- MAIN PROCESS ----////////////////////////////////////////////////////////////////////////////

  if (start_stage2)  {
    START1 = mtr_state = false;

    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);


    fan_1 = 2;
    sprinkler_1 = 2;


    publishStateChange(m_F1, fan_1, "All M_F1 stop published ");
    publishStateChange(m_S1, sprinkler_1, "All M_S1 stop published ");


    logger.println("Stage 2 Initiated wait for 5 secs");
    stage2_rtc_set = stage2_started = true;
    delay(5000);
  }

  //---- STAGE 1 ----////////////////////////////////////////////////////////////////////////////
  if (START1 && !stage2_rtc_set && !STOP) {
    if (currentState != STAGE1) {
      controller.writeDigitalOutput(STAGE_1_IO, HIGH);  // Turn On the LED of Stage 1
      logger.println("Stage 1 Started");
      setStage(STAGE1);
      fan_1_timer = millis() - (stage1_params.fanOnTime * MINS);
    }
    handleStage1();
  }

  //---- STAGE 2 ----////////////////////////////////////////////////////////////////////////////
  if (stage2_rtc_set && !stage3_started && !STOP) {
    if (currentState != STAGE2) {
      controller.writeDigitalOutput(STAGE_2_IO, HIGH);  // Turn On the LED of Stage 2

      logger.println("Stage 2 Started");
      setStage(STAGE2);
      fan_1_stg_2_timmer = millis() - (stage2_params.fanOffTime * MINS);
    }

    handleStage2();
  }

  //---- STAGE 3 ----////////////////////////////////////////////////////////////////////////////
  if (start_stage3) {
    START1 = START2 = stage2_rtc_set = mtr_state = false;

    // Turn All Output OFF
    controller.writeAnalogOutput(AIR_PWM, 0);
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);


    Output = 0;
    coef_output = 0;

    fan_1 = 2;  // When M_F1 = 2 ==> OFF
    sprinkler_1 = 2;  // When M_S1 = 2 ==> OFF


    publishStateChange(m_F1, fan_1, "Stage 3 F1 init published ");
    publishStateChange(m_S1, sprinkler_1, "Stage 3 S1 init published ");


    sprinkler_1_state = false;  // Put the all the states to 0
    logger.println("Stage 3 Initiated");
    stage3_started = true;
  }

  if (stage3_started && stage2_started && !STOP) {
    // State of Stage 3 turned to 1
    if ( currentState != STAGE3) {
      controller.writeDigitalOutput(STAGE_3_IO, HIGH);  // Turn ON the LED of Stage 3

      logger.println("Stage 3 Started");
      setStage(STAGE3);
      fan_1_stg_3_timer = millis() - (stage3_params.fanOffTime * MINS);
    }

   handleStage3();
  }
}

bool shouldStage2Start(DateTime &current_date) {
  bool is_after_stage2_time = current_date.hour() >= stage2_hour && current_date.minute() >= stage2_minute;
  bool is_after_stage_2_date = current_date.day() >= stage2_day && current_date.month() >= stage2_month;
  bool is_after_stage_2_start = is_after_stage2_time && is_after_stage_2_date;
  bool is_stage_2_triggered = (is_after_stage_2_start && START1) || START2;
  bool is_stage_2_ready = !stage2_started && !stage2_rtc_set;

  return is_stage_2_triggered && is_stage_2_ready;
}

bool shouldStage3Start(DateTime &current_date) {
  bool isReadyForStage3 = TS_F >= temp_set.ts && TC_F >= temp_set.tc;
  bool isStage3NotStarted = !stage3_started;
  bool isStage2Started = stage2_started;

  return isReadyForStage3 && isStage3NotStarted && isStage2Started;
}

void handleInputs() {
  //---- START, DELAYED, STOP Button pressed ----////////////////////////////////////////////////
  // delayed start push button or digital button pressed
  if (controller.readDigitalInput(DLY_S_IO) || remote_d_start) {
    START1 = true;
    logger.println("Delayed Start Pressed");
    remote_d_start = false;
    
    fan_1 = 2;
    sprinkler_1 = 2;

    publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 stop published ");
    publishStateChange(m_S1, sprinkler_1, "Stage 1 init M_S1 stop published ");
  }

  // start push button or digital button pressed
  if (controller.readDigitalInput(START_IO) || remote_start) {
    START2 = true;
    logger.println("Start Pressed");
    remote_start = false;
    fan_1 = 2;
    sprinkler_1 = 2;

    publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 stop published ");
    publishStateChange(m_S1, sprinkler_1, "Stage 1 init M_S1 stop published ");
  }

  // stop push button or digital button pressed
  if (controller.readDigitalInput(STOP_IO) || remote_stop) {
    STOP = true;
    logger.println("Stop Pressed");
    remote_stop = false;
  }
}

void handleStage1() {
    // Turn ON F1
  if (!mtr_state && !controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_timer, stage1_params.fanOffTime , true)) {
    controller.writeDigitalOutput(FAN_IO, HIGH);                                                                                       // Turn ON F1
    logger.println("Stage 1 F1 On");
    mtr_state = true;
    fan_1 = 1;  // When M_F1 = 1 ==> ON

    publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 ON published ");
  }

  // Turn OFF F1 when the time set in the configuration is over
  if (mtr_state && controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_timer, stage1_params.fanOnTime , true)) {
    controller.writeDigitalOutput(FAN_IO, LOW);
    // controller.writeAnalogOutput(AIR_PWM, 0);
    logger.println("Stage 1 F1 Off");
    mtr_state = false;
    fan_1 = 2;  // When M_F1 = 2 ==> OFF

    publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 OFF published ");
  }
}

void handleStage2() {
  // Turn ON F1 when time is over
  if (!mtr_state && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOffTime, true)) {
    controller.writeDigitalOutput(FAN_IO, HIGH);  // Output of F1
    logger.println("Stage 2 F1 On");
    mtr_state = true;
    fan_1 = 1;  // When M_F1 = 1 ==> ON

    publishStateChange(m_F1, fan_1, "Stage 2 F1 Start published ");
  }

  // Turn OFF F1 when time is over
  if (mtr_state && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOnTime, true) ){
    controller.writeDigitalOutput(FAN_IO, LOW);
    logger.println("Stage 2 F1 Off");
    mtr_state = false;
    fan_1 = 2;  // When M_F1 = 2 ==> OFF

    publishStateChange(m_F1, fan_1, "Stage 2 F1 Stop published ");
  }

  // Turn ON S1 when time is over
  if (mtr_state && !sprinkler_1_state && hasIntervalPassed(sprinkler_1_stg_2_timer,stage2_params.sprinklerOffTime, true)) {    
    controller.writeDigitalOutput(VALVE_IO, HIGH);  // Output of S1
    sprinkler_1_state = true;
    logger.println("Stage 2 S1 ON");
    sprinkler_1 = 1;  // When M_S1 = 1 ==> ON

    publishStateChange(m_S1, sprinkler_1, "Stage 2 S1 Start published ");
  }

  // Turn OFF S1 when time is over
  unsigned long timeConditionMet = hasIntervalPassed(sprinkler_1_stg_2_timer,stage2_params.sprinklerOnTime, true);

  if ((sprinkler_1_state || !mtr_state) && timeConditionMet) {    
    controller.writeDigitalOutput(VALVE_IO, LOW);  // Output of S1
    sprinkler_1_state = false;
    logger.println("Stage 2 S1 OFF");
    sprinkler_1 = 2;  // When M_S1 = 2 ==> OFF

    publishStateChange(m_S1, sprinkler_1, "Stage 2 S1 Stop published ");
  }

  // Calculate the Setpoint every 3 seconds in Function of Ta with the formula : Setpoint = A*(B-Ta)
  if (hasIntervalPassed(pid_computing_timer, 3000)) {
    Setpoint = (-(room.A * (temp_data.avg_ts)) + room.B);  //use the average of the temperature over the x last minuites
    pid_setpoint = float(Setpoint);

    mqtt.publishData(SETPOINT, pid_setpoint);

    logger.println("Setpoint published");
  }

  // Activate the PID when F1 ON
  if (mtr_state && hasIntervalPassed(turn_on_pid_timer, 3000)) {
    pid_input = TA_F;
    coef_output = (coef_pid * Output) / 100;  // Transform the Output of the PID to the desired max value
    logger.println(String(coef_output));
    air_in_feed_PID.Compute();
    // analogWrite(A0_5, Output);
    controller.writeAnalogOutput(AIR_PWM, Output);
    Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
    logger.println("Converted_Output is " + String(Converted_Output));
  }

  // Put the PID at 0 when F1 OFF
  if (!mtr_state && hasIntervalPassed(turn_off_pid_timer, 3000)) {
    //Setpoint = 0;
    pid_input = 0;
    Output = 0;
    coef_output = 0;
    // analogWrite(A0_5, Output);
    controller.writeAnalogOutput(AIR_PWM, Output);
    Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
    logger.println("Converted_Output is " + String(Converted_Output));
  }
}

void handleStage3() {
  // Turn ON F1 when time is over
  if(!mtr_state && hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOffTime, true)) {
    controller.writeDigitalOutput(FAN_IO, HIGH);
    // controller.writeAnalogOutput(AIR_PWM, duty_cycle);
    logger.println("Stage 3 F1 On");
    mtr_state = true;
    fan_1 = 1;

    publishStateChange(m_F1, fan_1, "Stage 3 F1 start published ");
  }

  // Turn OFF F1 when time is over
  if(mtr_state && hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOnTime, true)) {
    controller.writeDigitalOutput(FAN_IO, LOW);
    // controller.writeAnalogOutput(AIR_PWM, 0);
    logger.println("Stage 3 F1 Off");
    mtr_state = false;
    fan_1 = 2;

    publishStateChange(m_F1, fan_1, "Stage 3 F1 stop published ");
  }

  if (!sprinkler_1_state && hasIntervalPassed(sprinkler_1_stg_3_timer, stage3_params.sprinklerOffTime, true)) {
    controller.writeDigitalOutput(VALVE_IO, HIGH);
    sprinkler_1_state = true;
    logger.println("Stage 3 S1 ON");
    sprinkler_1 = 1;

    publishStateChange(m_S1, sprinkler_1, "Stage 3 S1 start published ");
  }

  if (sprinkler_1_state && hasIntervalPassed(sprinkler_1_stg_3_timer, stage3_params.sprinklerOnTime, true)) {
    controller.writeDigitalOutput(VALVE_IO, LOW);
    sprinkler_1_state = false;
    logger.println("Stage 3 S1 OFF with value of S1 ");
    sprinkler_1 = 2;

    publishStateChange(m_S1, sprinkler_1, "Stage 3 S1 stop published ");
  }
}
//// fct Callback ==> RECEIVE MQTT MESSAGES ////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int len) {
  logger.println("Message arrived [" + String(topic) + "]");

  // Delayed start timing
  if (mqtt.isTopicEqual(topic, sub_hours) && noButtonPressed()) {
    stage2_hour = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Hours set to: " + String(stage2_minute));
  }

  if (mqtt.isTopicEqual(topic, sub_minutes) && noButtonPressed()) {
    stage2_minute = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Minutes set to: " + String(stage2_minute));
  }

  if (mqtt.isTopicEqual(topic, sub_day) && noButtonPressed()) {
    stage2_day = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Day set to: " + String(stage2_day));
  }

  if (mqtt.isTopicEqual(topic, sub_month) && noButtonPressed()) {
    stage2_month = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Month set to: " + String(stage2_month));
  }

  bool update_default_parameters = false;

  //F1 stg1 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st1_ontime) && noButtonPressed()) {
    stage1_params.fanOnTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 1 on time set to: " + String(stage1_params.fanOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st1_offtime) && noButtonPressed()) {
    stage1_params.fanOffTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 1 off time set to: " + String(stage1_params.fanOffTime) + " MINS");
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 2 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st2_ontime) && noButtonPressed()) {
    stage2_params.fanOnTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 2 on time set to: " + String(stage2_params.fanOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st2_offtime) && noButtonPressed()) {
    stage2_params.fanOffTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 2 off time set to: " + String(stage2_params.fanOffTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st2_ontime) && noButtonPressed()) {
    stage2_params.sprinklerOnTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 2 on time set to: " + String(stage2_params.sprinklerOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st2_offtime) && noButtonPressed()) {
    stage2_params.sprinklerOffTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 2 off time set to: " + String(stage2_params.sprinklerOffTime) + " MINS");
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 3 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st3_ontime) && noButtonPressed()) {
    stage3_params.fanOnTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 3 on time set to: " + String(stage3_params.fanOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st3_offtime) && noButtonPressed()) {
    stage3_params.fanOffTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 3 off time set to: " + String(stage3_params.fanOffTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st3_ontime) && noButtonPressed()) {
    stage3_params.sprinklerOnTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 3 on time set to: " + String(stage3_params.sprinklerOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st3_offtime) && noButtonPressed()) {
    stage3_params.sprinklerOffTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 3 off time set to: " + String(stage3_params.sprinklerOffTime) + " MINS");
    update_default_parameters = true;
  }

  // Sub A and Sub B value update
  if (mqtt.isTopicEqual(topic, sub_A) && noButtonPressed()) {
    room.A = mqtt.responseToFloat(payload, len);
    // room.A = atoi((char *)payload);
    logger.println("A set to: " + String(room.A));
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_B) && noButtonPressed()) {
    room.B = mqtt.responseToFloat(payload, len);
    logger.println("B set to: " + String(room.B));
    update_default_parameters = true;
  }

  // PID update
  if (mqtt.isTopicEqual(topic, sub_P) && noButtonPressed()) {
    Kp = mqtt.responseToFloat(payload, len);
    logger.println("P set to: " + String(Kp));
    R_P = true;
  }

  if (mqtt.isTopicEqual(topic, sub_I) && noButtonPressed()) {
    Ki = mqtt.responseToFloat(payload, len);
    logger.println("I set to: " + String(Ki));
    R_I = true;
  }

  if (mqtt.isTopicEqual(topic, sub_D) && noButtonPressed()) {
    Kd = mqtt.responseToFloat(payload, len);
    logger.println("D set to: " + String(Kd));
    R_D = true;
  }

  if (R_P && R_I && R_D && noButtonPressed()) {
    air_in_feed_PID.SetTunings(Kp, Ki, Kd);
    logger.println("New PID parameter updated");
    R_P = R_I = R_D = false;
  }

  if (mqtt.isTopicEqual(topic, sub_coefPID) && noButtonPressed()) {
    coef_pid = mqtt.responseToInt(payload, len);
    logger.print("coef PID : " + String(coef_pid));
  }

  // Target temperature Ts & Tc update
  if (mqtt.isTopicEqual(topic, sub_ts_set) && noButtonPressed()) {
    temp_set.ts = mqtt.responseToFloat(payload, len);
    logger.println("Ts Condition set to: " + String(temp_set.ts));
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_tc_set) && noButtonPressed()) {
    temp_set.tc = mqtt.responseToFloat(payload, len);
    // Tc_cond = temp_set->tc_set;
    logger.println("Tc Condition set to: " + String(temp_set.tc));
    update_default_parameters = true;
  }

  // START
  if (mqtt.isTopicEqual(topic, sub_start) && noButtonPressed()) {
    remote_start = mqtt.responseToInt(payload, len);
    logger.println("START BUTTON PRESSED ON NODE RED" + String(remote_start));
  }

  // D_START
  if (mqtt.isTopicEqual(topic, sub_d_start) && !START2 && !STOP) {
    remote_d_start = mqtt.responseToInt(payload, len);
    logger.println("d_start BUTTON PRESSED ON NODE RED" + String(remote_d_start));
  }

  // STOP
  if (mqtt.isTopicEqual(topic, sub_stop)) {
    remote_stop = mqtt.responseToInt(payload, len);
    logger.println("stop BUTTON PRESSED ON NODE RED" + String(remote_stop));
  }

  // Choose TS
  if (mqtt.isTopicEqual(topic, sub_chooseTs)) {
    chooseTs = mqtt.responseToInt(payload, len);
    logger.println("Ts is now IR" + String(chooseTs));
  }

  if(update_default_parameters) controller.updateDefaultParameters(stage1_params, stage2_params, stage3_params, room, temp_set);
}

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

    Output = 0;
    coef_output = 0;
    stop_temp1 = true;

    fan_1 = sprinkler_1 = 2;

    mqtt.publishData(m_F1, fan_1);
    mqtt.publishData(m_S1, sprinkler_1);
    setStage(IDLE);
  }

  if (!stop_temp2) {
    mtr_state = sprinkler_1_state = START1 = START2 = stage2_started = stage3_started = stage2_rtc_set = false;
    stop_temp2 = true;
  }

  if (stop_temp2) {
    logger.println("PROCESS STOPPED");
    stop_temp1 = stop_temp2 = STOP = false;
  }
}

bool isValidTemperature(float temp, float minTemp, float maxTemp, const String& sensorName) {
  bool is_valid = temp < minTemp || temp > maxTemp;
  if(is_valid) sendTemperaturaAlert(temp, sensorName);

  return is_valid;
}

void updateTemperature() {
  controller.updateProbesTemperatures();

  TA = controller.readTempFrom(TA_AI);
  TS = controller.readTempFrom(TS_AI);
  TC = controller.readTempFrom(TC_AI);

  // float ta_raw = controller.readTempFrom(TA_AI);
  // TA = isValidTemperature(ta_raw, TA_MIN, TA_MAX, "TA") ? ta_raw : TA_DEF;

  // float ts_raw = controller.readTempFrom(TS_AI);
  // TS = isValidTemperature(ts_raw, TS_MIN, TS_MAX, "TS") ? ts_raw : TS_DEF;

  // float tc_raw = controller.readTempFrom(TC_AI);
  // TC = isValidTemperature(tc_raw, TC_MIN, TC_MAX, "TC") ? tc_raw : TC_DEF;

  TI = controller.getOneWireTempFrom(controller.ADDRESS_TI);  // Assuming TI doesn't need validation
}

void sendTemperaturaAlert(float temp, String sensor){
  const String msg = "{\"temp\":" + String(temp) + ", \"sensor\":" + sensor + "}";
  mqtt.publishData(SPOILED_SENSOR, msg);
}

void setStage(SystemState Stage) {
  currentState = Stage;
  mqtt.publishData(STAGE, Stage);
}

bool noButtonPressed(){
  return !START1 && !START2 && !STOP;
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

void publishStateChange(const char* topic, int state, const String& message) {
  mqtt.publishData(topic, state);
  logger.println(message);
}

void aknowledgementRoutine(){
  mqtt.publishData(STAGE, currentState);
  mqtt.publishData(m_F1, fan_1);
  // mqtt.publishData(m_F2, fan_2);
  mqtt.publishData(m_S1, sprinkler_1);
  mqtt.publishData(PID_OUTPUT, coef_output);

    // STAGE 1
  mqtt.publishData(ACK_F1_ST1_ONTIME, stage1_params.fanOnTime);
  mqtt.publishData(ACK_F1_ST1_OFFTIME, stage1_params.fanOffTime);

  // STAGE 2
  mqtt.publishData(ACK_F1_ST2_ONTIME, stage2_params.fanOnTime);
  mqtt.publishData(ACK_F1_ST2_OFFTIME, stage2_params.fanOffTime);
  mqtt.publishData(ACK_S1_ST2_ONTIME, stage2_params.sprinklerOnTime);
  mqtt.publishData(ACK_S1_ST2_OFFTIME, stage2_params.sprinklerOffTime);

  // STAGE 3
  mqtt.publishData(ACK_F1_ST3_ONTIME, stage3_params.fanOnTime);
  mqtt.publishData(ACK_F1_ST3_OFFTIME, stage3_params.fanOffTime);
  mqtt.publishData(ACK_S1_ST3_ONTIME, stage3_params.sprinklerOnTime);
  mqtt.publishData(ACK_S1_ST3_OFFTIME, stage3_params.sprinklerOffTime);

  // A & B
  mqtt.publishData(ACK_A, room.A);
  mqtt.publishData(ACK_B, room.B);
}

void getTsAvg() {
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

  mqtt.publishData(AVG_TS_TOPIC, temp_data.avg_ts);
}

void publishPID(){
  if (hasIntervalPassed(stg_2_pid_timer, TIME_ACQ_DELAY + 1)) {
    logger.println("Soft PID Actual Output is" + String(Output));
    const float output_float = float(coef_output);
    pid_output = ((output_float - 0) / (255 - 0)) * (100 - 0) + 0;
    logger.println("PID Output /100 is" + String(pid_output));

    mqtt.publishData(PID_OUTPUT, pid_output);
  }
}

void publishTemperatures(DateTime &current_date) {
    temp_data.ta = TA_F;
    temp_data.ts = TS_F;
    temp_data.tc = TC_F;
    temp_data.ti = TI_F;
    temp_data.avg_ts = avg_ts;

    mqtt.publishData(TA_TOPIC, temp_data.ta);
    mqtt.publishData(TS_TOPIC, temp_data.ts);
    mqtt.publishData(TC_TOPIC, temp_data.tc);
    mqtt.publishData(TI_TOPIC, temp_data.ti);

    // for debug purpose
    logger.println("Average: " + String(temp_data.avg_ts));
    logger.println("Stage: " + String(currentState));
    // logger.println(String(controller.readDigitalInput(DI0)));
    logger.println("Ts: " + String(TS));
    logger.println("TC: " + String(TC));
    logger.println("Ta: " + String(TA));
    // logger.println("Nstart: " + String(remote_start));
    // logger.println("Nstop: " + String(remote_stop));
    // logger.println("A variable: " + String(room.A));
    // logger.println("B variable: " + String(room.B));
    // logger.println("P variable: " + String(Kp));
    // logger.println("I variable: " + String(Ki));
    // logger.println("D variable: " + String(Kd));
    logger.println("setpoint raw: " + String(Setpoint));
    logger.println("setpoint: " + String(pid_setpoint));

    logger.printTime("Time:", current_date.hour(), current_date.minute(), current_date.day(), current_date.month());
    logger.printTime("Stage 2 Time:", stage2_hour, stage2_minute, stage2_day, stage2_month);
}