#include "filter.h"
#include <math.h>
#include <Arduino.h>

static bool initialized = false;

static uint32_t t_prev = 0;
static float maxAlt = -1e9f;

// ------------------- KF State -------------------
static float h_est = 0.0f;   // altitude (m)
static float v_est = 0.0f;   // vertical velocity (m/s)

// KF covariance P (2x2)
static float P00 = 10.0f, P01 = 0.0f;
static float P10 = 0.0f,  P11 = 10.0f;

// KF tuning
static float Q00 = 0.05f;   // altitude process noise
static float Q11 = 8.0f;    // velocity process noise
static float R   = 4.0f;    // baro measurement variance (m^2)

// MPU6050 accel scale: 4096 LSB/g for ±8g range
static const float ACC_LSB_PER_G = 4096.0f;
static const float G_MPS2 = 9.80665f;

void initSignals(const Data &d, FlightSignals &s) {
  initialized = true;

  t_prev = d.tms;

  // KF init
  h_est = d.alt_m;
  v_est = 0.0f;

  // covariance init
  P00 = 10.0f; P01 = 0.0f;
  P10 = 0.0f; P11 = 10.0f;

  maxAlt = d.alt_m;

  // outputs
  s.tms = d.tms;
  s.acc_g = 1.0f;
  s.alt_m = h_est;
  s.vel_mps = v_est;
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

  // dt guards (important esp with SD/Serial stalls)
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;

  // --- 1) Acc magnitude in g ---
  float axg = d.ax / ACC_LSB_PER_G;
  float ayg = d.ay / ACC_LSB_PER_G;
  float azg = d.az / ACC_LSB_PER_G;
  float acc_g = sqrtf(axg*axg + ayg*ayg + azg*azg);

  // --- 2) Approx vertical linear acceleration (m/s^2) ---
  // IMPORTANT:
  // In your hardware, IMU -X points UP (vertical).
  // So use (-axg) as "up-axis acceleration in g".
  // At rest: (-axg) ≈ +1g, so linear accel ≈ 0 after subtracting 1g.
  float a_mps2 = ((-axg) * G_MPS2) - G_MPS2;

  // =========================================================
  // KF PREDICT
  // x = [h, v]^T
  // =========================================================
  h_est = h_est + v_est * dt + 0.5f * a_mps2 * dt * dt;
  v_est = v_est + a_mps2 * dt;

  // P = FPF^T + Q , F=[1 dt; 0 1]
  // compute explicitly (fast, no matrices)
  float nP00 = P00 + dt*(P10 + P01) + (dt*dt)*P11;
  float nP01 = P01 + dt*P11;
  float nP10 = P10 + dt*P11;
  float nP11 = P11;

  P00 = nP00 + Q00;
  P01 = nP01;
  P10 = nP10;
  P11 = nP11 + Q11;

  // =========================================================
  // KF UPDATE (baro altitude measurement)
  // z = h + noise
  // =========================================================
  float z = d.alt_m;
  float y = z - h_est;       // innovation
  float S = P00 + R;         // innovation covariance
  if (S < 1e-6f) S = 1e-6f;

  float K0 = P00 / S;
  float K1 = P10 / S;

  h_est = h_est + K0 * y;
  v_est = v_est + K1 * y;

  // covariance update: P = (I - K H) P , H=[1 0]
  float P00_old = P00, P01_old = P01;
  float P10_old = P10, P11_old = P11;

  P00 = P00_old - K0*P00_old;
  P01 = P01_old - K0*P01_old;
  P10 = P10_old - K1*P00_old;
  P11 = P11_old - K1*P01_old;

  // --- 3) Track max altitude ---
  if (h_est > maxAlt) maxAlt = h_est;

  // Update history
  t_prev = now;

  // Output signals
  s.tms = now;
  s.acc_g = acc_g;
  s.alt_m = h_est;
  s.vel_mps = v_est;
  s.maxAlt_m = maxAlt;

  // Debug only (OK, but keep it OFF in actual flight)
  Serial.print("ACC: "); Serial.print(s.acc_g); Serial.print("|");
  Serial.print("Alt: "); Serial.print(s.alt_m); Serial.print("|");
  Serial.print("Vel: "); Serial.println(s.vel_mps);
}
