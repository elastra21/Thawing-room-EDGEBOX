#ifndef MY_SCREEN_H
#define MY_SCREEN_H
#include <M5Tough.h>

#define WATER_VALVE_CENTER 15
#define CIRC_FAN_CENTER 110
#define AIR_INFEED_CENTER 205
#define STAGE_CENTER 295

class Screen {
  public:
    void connecting();
    void printLabels();
    void updateStage(uint8_t stage);
    void updateTemperatures(float Ta, float Ts, float Tc, float Ti);
    void updateIndicators(int8_t water_valve = -1, int8_t circulation_fan = -1, int8_t air_in_feed = -1);
  private:
    
};
#endif
