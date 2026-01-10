#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include "data.h"

// Initialize GPS module
bool initGPS();

// Read GPS data
bool readGPS(double *lat, double *lon, float *alt);

// Optional: get last latitude/longitude
float getLatitude();
float getLongitude();

#endif
