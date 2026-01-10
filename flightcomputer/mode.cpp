#include <Arduino.h>
#include "mode.h"
#include "config.h"
#include "imu.h"
#include "pre.h"
#include "lora.h"
#include "sd.h"
#include "gps.h"

FlightMode currentMode = IDLE;
SystemHealth sysHealth = SYS_OK;

SystemHealth checkHealth() {

    // Example health logic:
    // Check if all sensors initialized properly
    // Return SYS_OK or SYS_HALT
    
    return SYS_OK;
}

\

void setMode(FlightMode newMode) {
    currentMode = newMode;
    Serial.print("Mode changed to: ");
    Serial.println(newMode);
    // Optional: log this event to SD/LoRa
}

void modeManager(void *pvParameters) {
    for (;;) {
        switch (currentMode) {
            case IDLE:
                if (checkCmd() == "ARM") {
                    // led blink: for LoRa connection & waiting for LAUNCH cmd
                    // sent status through lora
                    initialize();
                    setMode(INIT);
                }
                break;

            case INIT:
                if (initialize() && checkHealth() == SYS_OK) {
                    // send system health "SYS_OK msg"
                    
                    String cmd = checkCmd();
                    if (cmd == "LAUNCH") {
                        setMode(LAUNCH);
                    }
                }
                 break;

            case LAUNCH:
                // stste handled by freeRTOS
                break;

            case RECOVERY:
                digitalWrite(PARA_PIN, HIGH);
                digitalWrite(PAYLOAD_PIN,HIGH); // deploy parachute
                // After some condition → setMode(RETRIEVAL)
                break;

            case RETRIEVAL:
                //power saving mode ON
                //disable IMU+pre etc,only LoRa,GPS works
                // Idle until reset/power off
                break;

            default:
                sysHealth = SYS_HALT;
                break;
        }

        
    }
}

bool initialize() {
    return initIMU() &&
           initPressureSensor() &&
           initLoRa() &&
           initSD(SD_CS) &&
           initGPS();
}