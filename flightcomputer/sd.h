#ifndef SDCARD_H
#define SDCARD_H
#include <Arduino.h>
#include "config.h"
#include "filter.h"





bool initSD();
bool logData(const Data& d,const FlightSignals& s);

void closeLog();
void flushLog();

#endif