#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "sd.h"
#include "config.h"
#include "data.h"

// Use SdFat instead of SD.h
SdFat SD;
SdFile logFile;

// Initialize SD card
bool initSD(uint8_t SD_CS_pin) {
    if (!SD.begin(SD_CS_pin, SPI_HALF_SPEED)) { // SPI_HALF_SPEED works for most boards
        Serial.println("SD init failed!");
        return false;
    }

    // Create file with header if not exists
    if (!SD.exists("/log.csv")) {
        if (!logFile.open("/log.csv", O_RDWR | O_CREAT | O_AT_END)) {
            Serial.println("Failed to create log.csv");
            return false;
        }
        logFile.println("time,ax,ay,az,gx,gy,gz,pressure,temperature,alti");
        logFile.close();
    }

    return true;
}

// Log data in CSV format
void logData(const SensorData &data) {
    if (!logFile.open("/log.csv", O_RDWR | O_CREAT | O_AT_END)) return;

    logFile.print(data.time); logFile.print(",");
    logFile.print(data.ax); logFile.print(",");
    logFile.print(data.ay); logFile.print(",");
    logFile.print(data.az); logFile.print(",");
    logFile.print(data.gx); logFile.print(",");
    logFile.print(data.gy); logFile.print(",");
    logFile.print(data.gz); logFile.print(",");
    logFile.print(data.pre); logFile.print(",");
    logFile.print(data.temp); logFile.print(",");
    logFile.println(data.alti);

    logFile.close();
}
