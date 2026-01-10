#pragma once
#include <Arduino.h>
#include "data.h"

bool initSD(uint8_t SD_CS);
void logData(const SensorData &data);
