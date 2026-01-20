#include "sensors.h"
#include <MPU6050.h>
#include "config.h"
#include <Adafruit_BMP085.h>
#include <Arduino.h>
#include <Wire.h>

static MPU6050 mpu;
static Adafruit_BMP085 bmp;

static bool imuOK = false;
static bool bmpOK = false;

static int32_t pre0_pa = 101325;   
static float   alt0_m  = 0.0f;     

static int16_t ax_off=0, ay_off=0, az_off=0;
static int16_t gx_off=0, gy_off=0, gz_off=0;

static int32_t seaLevelPa = 101325; // can calibrate later

void calibrateSensor() {
  if(!imuOK) return;

  const int N = 500;
  int32_t ax_sum=0, ay_sum=0, az_sum=0;
  int32_t gx_sum=0, gy_sum=0, gz_sum=0;

  

  int16_t ax, ay, az, gx, gy, gz;

  for(int i=0; i<N; i++){
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    delay(5);
  }

  ax_off = ax_sum / N;
  ay_off = ay_sum / N;
  az_off = (az_sum / N) - 4096;  // assumes ±2g range and Z sees +1g at rest

  gx_off = gx_sum / N;
  gy_off = gy_sum / N;
  gz_off = gz_sum / N;

  if (bmpOK) {
    const int M = 50;
    int64_t p_sum = 0;
    float alt_sum = 0;

    for (int i = 0; i < M; i++) {
      int32_t p = bmp.readPressure();
      float alt = bmp.readAltitude((float)seaLevelPa);

      p_sum += p;
      alt_sum += alt;
      delay(10);
    }

    pre0_pa = (int32_t)(p_sum / M);
    alt0_m  = alt_sum / M;
  }
}

const char* FlightModeStr[] = {"IDLE","ASCENT","APOGEE","DESCENT","LANDED"};

bool initBMP() {
  bmpOK = bmp.begin();   // <-- set the module-local flag
  return bmpOK;
}

bool initIMU() {
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  imuOK = mpu.testConnection();
  return imuOK;
}


bool readIMU(Data &d) {
  if(!imuOK) return false;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  d.ax = ax - ax_off;
  d.ay = ay - ay_off;
  d.az = az - az_off;
  d.gx = gx - gx_off;
  d.gy = gy - gy_off;
  d.gz = gz - gz_off;
  Serial.print("MODE: ");
  Serial.println(FlightModeStr[(int)d.mode]);
  // Serial.print("AX: "); Serial.print(d.ax);
  // Serial.print(" AY: "); Serial.print(d.ay);
  // Serial.print("  |  ");
  // Serial.print(" AZ: "); Serial.println(d.az);

  //Serial.print(" | GX: "); Serial.print(d.gx);
  //Serial.print(" GY: "); Serial.print(d.gy);
  //Serial.print(" GZ: "); Serial.println(d.gz);

  return true;
}


bool readBMP(Data &d) {
  if(!bmpOK) return false;

  int32_t p = bmp.readPressure();
  float alt = bmp.readAltitude((float)seaLevelPa);

  d.pre = p;          
  d.alt_m = alt - alt0_m;       // altitude relative to pad (meters)
  Serial.print("Pre: "); Serial.print(d.pre); Serial.print(" | ");
  Serial.print(("Alt: ")); Serial.print((d.alt_m));

  return true;
}




