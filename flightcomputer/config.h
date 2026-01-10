#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Flight modes
enum FlightMode {
  IDLE,
  INIT,
  LAUNCH,
  RECOVERY,
  RETRIEVAL
};

// System health states
enum SystemHealth {
  SYS_OK,
  SYS_HALT
};

// ------------------- Pin definitions -------------------
const int PARA_PIN = 25;   // Parachute trigger
//GPIO 3
//GPIO 46
//GPIO 10
//GPIO 48 
//GPIO 47 Green
//GPIO 23 Blue
const int PAYLOAD_PIN  = 26;   // Payload trigger
const int SD_CS    = 5;    // SD card CS


const int LORA_SS = 18;
const int LORA_RST = 14;
const int LORA_DIO0 = 26;

const long LORA_FREQUENCY = 433E6;


const int GPS_RX = 16;
const int GPS_TX = 17;

// ------------------- Global states -------------------
extern FlightMode currentMode ;
extern SystemHealth sysHealth ;

#endif
