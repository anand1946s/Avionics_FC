#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <LoRa.h>
#include "config.h"
#include "data.h"

bool initLoRa();
void sendCommand(const String &cmd);
String checkCmd();      // needed for mode.cpp
String receiveData();
bool sendData(const SensorData &data);

#endif
