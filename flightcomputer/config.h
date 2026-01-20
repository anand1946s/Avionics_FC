#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

enum FlightMode {
  IDLE,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED
};


typedef struct Data{
  uint32_t tms;// in ms
  int16_t pre;
  float alt_m;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  FlightMode mode;

} Data;




const int PAYLOAD_PIN = 4;
const int PARA_PIN = 5;
const int SD_CS = 10;
const int BUZZER_PIN = 11;

#endif