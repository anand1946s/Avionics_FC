#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "pre.h"
#include "lora.h"
#include "sd.h"
#include "gps.h"
#include "mode.h"
#include "config.h"

// Queues for inter-task communication
QueueHandle_t loggerQueue;
QueueHandle_t telemetryQueue;

// ------------------- TASKS ----------------------

// Sensor reading task
void sensorTask(void *pvParameters) {
    SensorData data;
    const TickType_t delayTicks = 20 / portTICK_PERIOD_MS; // 50 Hz

    for (;;) {
        data.time = millis();
        data.lat = NAN;
        data.lon = NAN;
        data.gpsAlt = NAN;

        readIMU(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);
        
        readPressure(&data.pre, &data.temp);
        data.alti = calculateAltitude(data.pre);

        // Send to both queues
        xQueueSend(loggerQueue, &data, 0);
        xQueueSend(telemetryQueue, &data, 0);

        vTaskDelay(delayTicks);
    }
}

// Logger task
void loggerTask(void *pvParameters) {
    SensorData data, tmp;

    for (;;) {
        if (xQueueReceive(loggerQueue, &data, 0) == pdTRUE) {
            while (xQueueReceive(loggerQueue, &tmp, 0) == pdTRUE) {
                data = tmp; // keep latest
            }
            logData(data);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Telemetry task
void telemetryTask(void *pvParameters) {
    const TickType_t delayTicks = 1000 / portTICK_PERIOD_MS;
    SensorData data;

    for (;;) {
        if (xQueueReceive(telemetryQueue, &data, 0) == pdTRUE) {
            SensorData tmp;
            while (xQueueReceive(telemetryQueue, &tmp, 0) == pdTRUE) {
                data = tmp;
            }

            if (currentMode == RECOVERY) {
                double lat, lon;
                float gpsAlt;
                if (readGPS(&lat, &lon, &gpsAlt)) {
                    data.lat = lat;
                    data.lon = lon;
                    data.gpsAlt = gpsAlt;
                }
            }

            sendData(data);
        }
        vTaskDelay(delayTicks);
    }
}

// Command task
void commandTask(void *pvParameters) {
    const TickType_t delayTicks = 100 / portTICK_PERIOD_MS;
    for (;;) {
        String c = checkCmd();
        // Don’t call modeManager() here → it runs as its own FreeRTOS task
        vTaskDelay(delayTicks);
    }
}

// ------------------- SETUP ----------------------

void setup() {
    Wire.begin();
    Serial.begin(115200);

    // Create queues
    loggerQueue    = xQueueCreate(10, sizeof(SensorData));
    telemetryQueue = xQueueCreate(10, sizeof(SensorData));

    // Create tasks
    xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 2, NULL, 1);
    //xTaskCreatePinnedToCore(loggerTask, "Logger", 4096, NULL, 1, NULL, 1);
    //xTaskCreatePinnedToCore(telemetryTask, "Telemetry", 4096, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(commandTask, "Command", 4096, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(modeManager, "Mode", 4096, NULL, 2, NULL, 0);
}

void loop() {
    // Nothing here tbh, Freertos handles tasks
}
