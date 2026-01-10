// we write the methods(functions) like imu reading .calibration also the complimentary filters we be using here.
// can use libraries as esp32 ha enough memory.
// may also try using wire library(bare metal i2c coding)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>

// Offsets for calibration
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;

// Globals for filtered angles (used in filters)
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

float dt = 0.02;  // if imu rate is 20ms. or declare this in sensortask and dlt this





Adafruit_LSM6DSO32 imu;


bool initIMU() {
    

    // Try to initialize IMU
    if (!imu.begin_I2C()) {  
        //Serial.println("Failed to find LSM6DSO32!");
        return false;
    }

    // Configure Accelerometer
    imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G); // ±16g (enough for rockets)
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);      // ~100 Hz sampling

    // Configure Gyroscope
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);  // Max angular velocity
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);       // Match accel rate

    // Optional: filter setup
    //imu.setFilterBandwidth(LSM6DS_FILTER_HZ_50);   // smooth signals

    //Serial.println("LSM6DSO32 Initialized");
    return true;
}

void calibrateIMU() {
    const int samples = 200;   // number of samples to average
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;

    sensors_event_t accel, gyro, temp;

    // Collect samples
    for (int i = 0; i < samples; i++) {
        imu.getEvent(&accel, &gyro, &temp);

        gx += gyro.gyro.x;
        gy += gyro.gyro.y;
        gz += gyro.gyro.z;

        ax += accel.acceleration.x;
        ay += accel.acceleration.y;
        az += accel.acceleration.z;

        delay(5); // small delay between reads 
    }

    // Average values
    gx /= samples;
    gy /= samples;
    gz /= samples;

    ax /= samples;
    ay /= samples;
    az /= samples;

    // Save offsets
    gyroOffsetX = gx;
    gyroOffsetY = gy;
    gyroOffsetZ = gz;

    // For accel: we expect (0,0,1g) when flat
    accelOffsetX = ax;
    accelOffsetY = ay;
    accelOffsetZ = (az - 9.81);  // adjust for gravity (m/s^2)

    //Serial.println(" IMU calibration done");
}


void resetOrientation() {
    
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
}


void readIMU(float *ax, float *ay, float *az,
             float *gx, float *gy, float *gz) {
    sensors_event_t accel, gyro, temp;

    
    imu.getEvent(&accel, &gyro, &temp);

    // Calib offsets
    *gx = gyro.gyro.x - gyroOffsetX;
    *gy = gyro.gyro.y - gyroOffsetY;
    *gz = gyro.gyro.z - gyroOffsetZ;

    *ax = accel.acceleration.x - accelOffsetX;
    *ay = accel.acceleration.y - accelOffsetY;
    *az = accel.acceleration.z - accelOffsetZ;
}

void filters(float ax, float ay, float az,
             float gx, float gy, float gz,
             float dt) {

    // evrything in radians
    float rollAcc  = atan2(ay, az);           
    float pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)); 

    
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;   

    // --- Complementary filter: blend gyro and accel ---
    const float alpha = 0.98;  // weight for gyro
    roll  = alpha * roll + (1.0 - alpha) * rollAcc;
    pitch = alpha * pitch + (1.0 - alpha) * pitchAcc;

    // Optional: normalize yaw between -PI and PI
    if (yaw > PI) yaw -= 2*PI;
    else if (yaw < -PI) yaw += 2*PI;
}


// bool checkEvent() {
//     // Example: if acceleration magnitude > threshold
//     float ax, ay, az, gx, gy, gz;
//     readIMU(&ax, &ay, &az, &gx, &gy, &gz);

//     float accMag = sqrt(ax*ax + ay*ay + az*az);
//     return (accMag > 15.0f); // returns true if > ~1.5g
// }


