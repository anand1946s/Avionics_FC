#ifndef STATES_H
#define STATES_H

#include "config.h"

extern FlightMode currentMode;

void modemanager(const FlightSignals& s);

#endif