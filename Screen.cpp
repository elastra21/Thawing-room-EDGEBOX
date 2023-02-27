#include "Screen.h"

void Screen::updateStage(uint8_t stage){
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(295, 20);
  M5.Lcd.print(stage);    
}

void Screen::printLabels(){
  M5.Lcd.print("Water Valve     Circ. fan     Air in-feed %     Stage");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(260, 0);
  M5.Lcd.setCursor(10, 100);
  M5.Lcd.print("Ta:");
  M5.Lcd.setCursor(10, 130);
  M5.Lcd.print("Ts:");
  M5.Lcd.setCursor(10, 160);
  M5.Lcd.print("Tc:");
  M5.Lcd.setCursor(10, 190);
  M5.Lcd.print("Ti:");
}

void Screen::updateTemperatures(float Ta, float Ts, float Tc, float Ti){
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(50, 100);
  M5.Lcd.printf("%.3f",Ta);
  M5.Lcd.setCursor(50, 130);
  M5.Lcd.printf("%.3f",Ts);
  M5.Lcd.setCursor(50, 160);
  M5.Lcd.printf("%.3f",Tc);
  M5.Lcd.setCursor(50, 190);
  M5.Lcd.printf("%.3f",Ti);
}

void Screen::updateIndicators(int8_t water_valve, int8_t circulation_fan, int8_t air_in_feed){
  const int8_t values[] = { water_valve, circulation_fan, air_in_feed };
  const uint8_t indicators_centers[] = { WATER_VALVE_CENTER, CIRC_FAN_CENTER, AIR_INFEED_CENTER};

  M5.Lcd.setTextSize(2);
  for(size_t i = 0; i < sizeof(indicators_centers); i++ ){
    if (values[i] > -1){
      String value_to_send = String(values[i]);
      if (i < 2) value_to_send = values[i] ? "ON" : "OFF";
      else if (i == 2) value_to_send += "%";
      M5.Lcd.setCursor(indicators_centers[i], 20);
      M5.Lcd.print(value_to_send);
    }
  }  
}

void Screen::connecting(){
    
}
