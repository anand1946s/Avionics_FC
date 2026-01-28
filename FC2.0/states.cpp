#include "config.h"
#include "sensors.h"
#include "filter.h"
#include "sd.h"
#include <Arduino.h>


#define LANDED_VEL_ABS_MPS    1.0f   // tune
#define LANDED_COUNT          200    // e.g. 200 cycles @ 50Hz = 4s

extern FlightMode currentMode;

static const float LIFTOFF_ACC_G = 2.0f;     // liftoff accel threshold
static const uint16_t LIFTOFF_COUNT = 3;    // sustained samples

static const float APOGEE_DROP_M =  1.5f;     // drop from maxAlt to confirm apogee
static const uint16_t APOGEE_COUNT = 8;      // sustained samples

// ------------ ACTUATION PINS ------------
static const uint16_t DEPLOY_MS = 500;       // pyro ON time (tune!)

static const uint16_t PAYLOAD_ON_MS = 1000;  // 1s
static const uint16_t PARA_ON_MS    = 1000;  // 1s


static void checkLift(const FlightSignals& s) {
  static uint16_t cnt = 0;

  if (s.acc_g > LIFTOFF_ACC_G) cnt++;
  else cnt = 0;

  if (cnt >= LIFTOFF_COUNT) {
    currentMode = ASCENT;
    cnt = 0;
  }
}

static void checkApogee(const FlightSignals& s) {
  static uint16_t dropCnt = 0;

  if (currentMode != ASCENT) return;


  if (s.maxAlt_m < 20.0f) return;

  
  if (s.vel_mps > -0.4f) {
    dropCnt = 0;
    return;
  }

  if (s.alt_m < (s.maxAlt_m - APOGEE_DROP_M)) dropCnt++;
  else dropCnt = 0;


  if (dropCnt >= APOGEE_COUNT) {
    currentMode = APOGEE;
    dropCnt = 0;
  }
}


void deploy() {
  static bool started = false;
  static uint32_t t0 = 0;
  static bool paraStarted = false;

  if (!started) {
    started = true;
    paraStarted = false; 
    t0 = millis();

    // payload immediately
    digitalWrite(PAYLOAD_PIN, HIGH);
  }

  uint32_t dt = millis() - t0;

  // stop payload after 1s
  if (dt >= PAYLOAD_ON_MS) {
    digitalWrite(PAYLOAD_PIN, LOW);
  }

  // start parachute after DEPLOY_MS
  if (!paraStarted && dt >= DEPLOY_MS) {
    paraStarted = true;
    digitalWrite(PARA_PIN, HIGH);
  }

  // stop parachute after 1s from its start
  if (paraStarted && dt >= (DEPLOY_MS + PARA_ON_MS)) {
    digitalWrite(PARA_PIN, LOW);
    currentMode = DESCENT;
  }
}

void recover(){
  static bool done = false;
  if (done) return;
  done = true;

  Serial.println("LANDED---");
  closeLog();
}

static void landed(const FlightSignals& s) {
  static uint16_t cnt = 0;

  // only meaningful in descent
  if (currentMode != DESCENT) {
    cnt = 0;
    return;
  }

  if (fabsf(s.vel_mps) < LANDED_VEL_ABS_MPS) cnt++;
  else cnt = 0;

  if (cnt >= LANDED_COUNT) {
    currentMode = LANDED;
    cnt = 0;
  }
}

void modemanager(const FlightSignals& s){
  switch(currentMode){
    case IDLE:
      checkLift(s); 
      break;

    case ASCENT:
      checkApogee(s);
      break;

    case APOGEE:
      deploy();
      break;

    case DESCENT:
      landed(s);
      break;

    case LANDED:
      recover();  // some buzzer noise etc
      break;
  }
}