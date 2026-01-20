#include "filter.h"
#include <math.h>
#include <Arduino.h>

static bool initialized = false;

static float alt_f = 0.0f;
static float vel_f = 0.0f;

static float alt_prev = 0.0f;
static uint32_t t_prev = 0;

static float maxAlt = -1e9;

// EMA coefficients (tune later)
static const float ALPHA_ALT = 0.25f;  // altitude smoothing
static const float ALPHA_VEL = 0.35f;  // velocity smoothing

// MPU6050 accel scale: 16384 LSB/g for ±8g range
static const float ACC_LSB_PER_G = 4096.0f;

void initSignals(const Data &d, FlightSignals &s) {
  initialized = true;

  t_prev = d.tms;
  alt_prev = d.alt_m;

  alt_f = d.alt_m;
  vel_f = 0.0f;
  maxAlt = d.alt_m;

  s.tms = d.tms;
  s.acc_g = 1.0f;
  s.alt_m = alt_f;
  s.vel_mps = 0.0f;
  s.maxAlt_m = maxAlt;
}


void updateSignals(const Data &d, FlightSignals &s) {
  if (!initialized) {
    initSignals(d, s);
    return;
  }

  // --- timestamp ---
  uint32_t now = d.tms;
  float dt = (now - t_prev) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;

  // --- 1) Acc magnitude in g ---
  float axg = d.ax / ACC_LSB_PER_G;
  float ayg = d.ay / ACC_LSB_PER_G;
  float azg = d.az / ACC_LSB_PER_G;
  float acc_g = sqrtf(axg*axg + ayg*ayg + azg*azg);

  // --- 2) Filter altitude ---
  alt_f = (1.0f - ALPHA_ALT) * alt_f + (ALPHA_ALT) * d.alt_m;

  // --- 3) Velocity estimate + filter ---
  float vel_raw = (alt_f - alt_prev) / dt;
  vel_f = (1.0f - ALPHA_VEL) * vel_f + (ALPHA_VEL) * vel_raw;

  // --- 4) Track max altitude ---
  if (alt_f > maxAlt) maxAlt = alt_f;

  // Update history
  alt_prev = alt_f;
  t_prev = now;

  // Output signals
  s.tms = now;
  s.acc_g = acc_g;
  s.alt_m = alt_f;
  s.vel_mps = vel_f;
  s.maxAlt_m = maxAlt;
  // Serial.print("T: "); Serial.print(s.tms);Serial.print("|");
  Serial.print("ACC: "); Serial.print(s.acc_g);Serial.print("|");
  Serial.print("Vel: "); Serial.println(s.vel_mps);
  

}