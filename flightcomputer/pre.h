#ifndef PRE_H
#define PRE_H

#include <Arduino.h>
#include "data.h"
// Function prototypes
bool initPressureSensor();
void readPressure(float *pressureVal, float *tempVal);
float calculateAltitude(float pressureVal, float seaLevelPressure = 101325.0);
void resetPressureSensor();

#endif
