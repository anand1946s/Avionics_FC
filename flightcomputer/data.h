// sensors.h
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

typedef struct {
  unsigned long time;
  float ax, ay, az;
  float gx, gy, gz;
  float pre;
  float temp;
  float alti;

  double lat;
  double lon;
  float gpsAlt;
} SensorData;

#endif
