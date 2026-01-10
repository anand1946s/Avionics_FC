#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "data.h"

// Function prototypes
bool initIMU();
void calibrateIMU();
void resetOrientation();

void readIMU(float *ax, float *ay, float *az,
             float *gx, float *gy, float *gz);

void filters(float ax, float ay, float az,
             float gx, float gy, float gz,
             float dt);

bool checkEvent();   // e.g., detect launch, anomalies, etc.

#endif
