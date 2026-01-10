#ifndef MODE_H
#define MODE_H

#include <Arduino.h>
#include "config.h"
// Flight modes


// Globals
extern FlightMode currentMode;
extern SystemHealth sysHealth;

// Functions
SystemHealth checkHealth();
bool checkLaunch();
void setMode(FlightMode newMode);
void modeManager(void *pvParameters);  // FreeRTOS task
bool initialize();

#endif
