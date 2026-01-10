#include <Arduino.h>
#include "pre.h"
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;

// --- Globals ---
float pre  = 0.0f;   // Pa
float temp = 0.0f;   // °C
float alti = 0.0f;   // meters

// --- Initialize sensor ---
bool initPressureSensor() {
    

    if (!bmp.begin_I2C()) {
        //Serial.println("Failed to find BMP3XX sensor!");
        return false;
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);


    alti = 0.0f;
    return true;
}

void readPressure(float *pressureVal, float *tempVal) {
    if (!bmp.performReading()) {
        // keep last good values
        *pressureVal = pre;
        *tempVal = temp;
        return;
    }

    pre = bmp.pressure;       // Pa
    temp = bmp.temperature;   // °C

    *pressureVal = pre;
    *tempVal = temp;
}

float calculateAltitude(float pressureVal, float seaLevelPressure) {
    alti = 44330.0f * (1.0f - pow(pressureVal / seaLevelPressure, 0.1903f));
    return alti;
}

void resetPressureSensor() {
    // Simply re-init
    initPressureSensor();
}
