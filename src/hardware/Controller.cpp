#include "Controller.h"

RTC_DS3231 rtc;
WiFiUDP ntpUDP;
// TwoWire rtc_i2c = TwoWire(0);
NTPClient timeClient(ntpUDP);
OneWire oneWire(ONE_WIRE_BUS);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature temp_sensor_bus(&oneWire);  // PASS our oneWire reference to Dallas Temperature.

const float voltage_per_step = REFERENCE / ADC__RESOLUTION;
const int16_t range = TEMPERATURE_MAX - TEMPERATURE_MIN;
const double temperature_per_step = range / REFERENCE;

Controller::Controller(/* args */) {
  setUpLogger();
  WebSerial.println("Controller created");
}

Controller::~Controller() {

}

void Controller::init() {
  setUpI2C();
  setUpIOS();
}

void Controller::setUpLogger() {
  #ifdef WebSerial
    WebSerial.begin(115200);
    WebSerial.println("Logger set up");
  #endif
}

void Controller::setUpIOS() {
  edgebox.init(); // Setup IO pins
   // setUpDigitalInputs();
  // setUpDigitalOutputs();
  setUpAnalogInputs();
  setUpAnalogOutputs();
  // Setup temperature sensors
  setUpOneWireProbes(); 

}


void Controller::setUpAnalogOutputs() {
  ledcSetup(AIR_PWM, FREQ, RESOLUTION);
  ledcAttachPin(AIR_PIN, AIR_PWM);
}

void Controller::setUpDigitalOutputs() {
  // for (uint8_t i = 0; i < outputs_size; i++) pinMode(outputs[i], OUTPUT);
}

void Controller::setUpDigitalInputs() {
  //Testing pourpose
  // pinMode(PORT_B0, INPUT_PULLUP);

  // for (uint8_t i = 0; i < inputs_size; i++) pinMode(inputs[i], INPUT_PULLUP);
}

void Controller::setUpAnalogInputs() {
    analog_inputs.begin();
}

void Controller::setUpOneWireProbes(){
  temp_sensor_bus.setResolution(ADDRESS_TA, TEMPERATURE_PRECISION);
  temp_sensor_bus.setResolution(ADDRESS_TS, TEMPERATURE_PRECISION);
  temp_sensor_bus.setResolution(ADDRESS_TC, TEMPERATURE_PRECISION);
  temp_sensor_bus.setResolution(ADDRESS_TI, TEMPERATURE_PRECISION);
}

void Controller::setUpI2C() {
  while (!Wire.begin()){
    WebSerial.println("RTC I2C not found");
    delay(1000);
  }

}

void Controller::setUpRTC() {
  if (!rtc.begin()) {
      WebSerial.println("Couldn't find RTC");
    while (1);
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  DateTime now = rtc.now();
  if (true) {
    Serial.println("RTC time seems invalid. Adjusting to NTP time.");
    
    timeClient.begin();
    timeClient.setTimeOffset(SECS_IN_HR * TIME_ZONE_OFFSET_HRS);
    timeClient.setUpdateInterval(SECS_IN_HR);
    timeClient.update();

    delay(1000); 

    long epochTime = timeClient.getEpochTime();

    // Convert received time from Epoch format to DateTime format
    DateTime ntpTime(epochTime);

    // Adjust RTC
    rtc.adjust(ntpTime);
  }
}

bool Controller::isRTCConnected() {
  return rtc.begin();
}

DateTime Controller::getDateTime() {
  return rtc.now();
}

uint64_t Controller::readAnalogInput(uint8_t input) {
  return analog_inputs.readADC_SingleEnded(input);;
}

bool Controller::readDigitalInput(uint8_t input) {
  return digitalRead(input);
}

void Controller::writeAnalogOutput(uint8_t output, uint8_t value) {
  ledcWrite(AIR_PWM, value);
}

void Controller::writeDigitalOutput(uint8_t output, uint8_t value) {
  digitalWrite(output, value);
}

float Controller::readTempFrom(uint8_t channel) {
  const uint16_t raw_voltage_ch = analogRead(channel); 
  // const float voltage_ch = (raw_voltage_ch * voltage_per_step);
  // Serial.println(voltage_ch);
  // const float temp = (voltage_ch * temperature_per_step) + TEMPERATURE_MIN;
  const float temp = raw_voltage_ch*0.0247 - 52.933; // ramp calculated with excel trhough manual calibration
  return temp;
}

void Controller::updateProbesTemperatures(){
  temp_sensor_bus.requestTemperatures();
}

float Controller::getOneWireTempFrom(DeviceAddress address) {
  return temp_sensor_bus.getTempC(address);
}

// WIFI CLASS

void Controller::connectToWiFi(bool web_server, bool web_serial, bool OTA) {
  wifi.connectToWiFi();
  if(OTA) wifi.setUpOTA();
  if(web_server) wifi.setUpWebServer(web_serial);
}

void Controller::reconnectWiFi() {
  wifi.reconnect();
}

bool Controller::isWiFiConnected() {
  return wifi.isConnected();
}

bool Controller::refreshWiFiStatus() {
  return wifi.refreshWiFiStatus();
}

bool Controller::getConnectionStatus() {
  return wifi.getConnectionStatus();
}

void Controller::loopOTA() {
  wifi.loopOTA();
}

void Controller::setUpWiFi(const char* ssid, const char* password, const char* hostname) {
  wifi.init(ssid, password, hostname);
}