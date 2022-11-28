/*
 * This library handle the preprocessing of IMU data, including
 * sensor initialization, Kalman filter and sensor fusion.
 *
 * Require:
 * Return:
 */

#ifndef _SENSOR_H
#define _SENSOR_H

#include <Arduino.h>

#include <../../include/configs.h>

#ifdef USE_PERIPHERAL_BMP280
#include "Adafruit_BMP280_simplified.h"
#define HPa 0x01
#define Pa 0x02
#include <SimpleKalmanFilter.h>
#endif

#ifdef USE_PERIPHERAL_MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#endif

#ifdef USE_GY91_MPU9250
// #include <MPU9250.h>
#include <SparkFunMPU9250-DMP.h>
#endif

#ifdef USE_GPS_NEO6M
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#endif

typedef struct fvec {
    float x;
    float y;
    float z;
} fvec_t;

enum ROCKET_POSE { ROCKET_UNKNOWN, ROCKET_RISING, ROCKET_FALLING };

class SENSOR
{
private:
#ifdef USE_GY91_MPU9250
    MPU9250_DMP imu;
#endif


#ifdef USE_PERIPHERAL_BMP280
    Adafruit_BMP280 bmp;  // I2C
    SimpleKalmanFilter *altitudeKalmanFilter;
#endif
    float altitude_bmp;
    float velocity_bmp;
    float temperature;
    float pressure_Pa;
    float pressure_HPa;

    float rate_bmp;

public:
    fvec_t acc, gyro, mag, gps;
    float altitude_estimate;
    float velocity_estimate;

    SENSOR();

    fvec_t getAcc();
    fvec_t getGyro();
    fvec_t getMag();
    fvec_t getGps();

    float getBmpAltitude();
    float getBmpVelocity();
    // float getTemperature();
    float getPressure(uint8_t);

    // float getGPS();

    bool init();
    bool init_imu();
    bool init_bmp();
    bool init_gps();

    // void calibrate_imu();
    void calibrate_bmp();
    // void calibrate_gps();

    void update_imu();
    bool update_bmp();
    // void update_gps();
    void update();

    // Filter
    float LPF(float, float, float, float);

    ROCKET_POSE pose;
};

#endif