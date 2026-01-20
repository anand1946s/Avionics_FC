#include "config.h"
#include "sd.h"
#include "filter.h"
#include "sensors.h"
#include "states.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

bool imuOK = false;
bool bmpOK = false;
bool sdOK  = false;

uint32_t lastFlush = 0;
const uint16_t FLUSH_DT = 2000; // 2s

uint32_t lastRead = 0;
uint32_t lastLog  = 0;

const uint16_t READ_DT = 20;   // read sensors every 20ms = 50Hz
const uint16_t LOG_DT  = 500;   // log every 100ms



FlightMode currentMode = IDLE;
Data d;
FlightSignals s;

void setup() {
  Wire.begin();
  Wire.setWireTimeout(25000, true); 
  Serial.begin(9600);
  delay(500);
  pinMode(PAYLOAD_PIN, OUTPUT);
  pinMode(PARA_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
 


  imuOK = initIMU();
  sdOK = initSD();
  bmpOK = initBMP(); 
  Serial.println("Calibrating...");
  if (imuOK) calibrateSensor();

  // get one initial reading before initSignals
  d.tms = millis();
  if (imuOK) readIMU(d);
  if (bmpOK) readBMP(d);

  initSignals(d, s);


  if (sdOK && imuOK && bmpOK) {
    Serial.println("SYS_OK... READY_FOR_LAUNCH");
    
  } else {
    Serial.print("FAILED_TO_BOOT: ");
    if (!imuOK) Serial.print("IMU_FAIL ");
    if (!bmpOK) Serial.print("BMP_FAIL ");
    if (!sdOK)  Serial.print("SD_FAIL ");
    Serial.println();
  }
}


void loop() {
  uint32_t now = millis();
  d.tms = now;
  d.mode = currentMode;

  if (now - lastRead >= READ_DT) {
    lastRead = now;

    if (imuOK) readIMU(d);
    if (bmpOK) readBMP(d);
    

    if (imuOK || bmpOK){
      updateSignals(d, s);
      modemanager(s);

    } 
    
  }

  

  if (sdOK && (now - lastLog >= LOG_DT)) {
    lastLog = now;
    logData(d,s);
    
    
  }
  if (sdOK && (now - lastFlush >= FLUSH_DT)) {
  lastFlush = now;
  flushLog();   
}
  
}