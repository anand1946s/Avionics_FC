#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

bool initIMU();
bool initBMP();

bool readIMU(Data &d);
bool readBMP(Data &d);

void calibrateSensor();
// Convenience: read all sensors (non-blocking design later)
bool updateSensors(Data &d);

#endif
