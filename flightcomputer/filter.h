#ifndef FILTER_H
#define FILTER_H

#include "config.h"

// Derived + filtered flight values used by state machine
typedef struct {
  uint32_t tms;

  float acc_g;       
  float alt_m;       
  float vel_mps;     
  float maxAlt_m;    
} FlightSignals;

void initSignals(const Data &d, FlightSignals &s);
void updateSignals(const Data &d, FlightSignals &s);

#endif
