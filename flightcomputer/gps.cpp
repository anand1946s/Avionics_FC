#include <Arduino.h>
#include "gps.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// GPS setup
HardwareSerial gpsSerial(1); // Use UART1
TinyGPSPlus gps;

// Globals for last known location
double latitude = 0.0;
double longitude = 0.0;

// Initialize GPS
bool initGPS() {
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
    delay(1000); // let GPS stabilize
    return true;
}

// Read GPS data (call repeatedly in a loop or FreeRTOS task)
bool readGPS(double *lat, double *lon, float *alt) {
    while (gpsSerial.available() > 0) {             // feed GPS data
        if (gps.encode(gpsSerial.read())) {        // returns true if a new valid sentence parsed
            *lat = gps.location.lat();             // get latitude as double
            *lon = gps.location.lng();             // get longitude as double
            *alt = gps.altitude.meters();          // get altitude in meters
            latitude = *lat;                       // update global last known location
            longitude = *lon;
            return true;
        }
    }
    return false;  // no new data
}

// Get last known coordinates
float getLatitude() { return latitude; }
float getLongitude() { return longitude; }
