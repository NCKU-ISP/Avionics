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
#include <SimpleKalmanFilter.h>
//#include "Adafruit_BMP280.h"
#endif

#ifdef USE_PERIPHERAL_MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#endif

#ifdef USE_GY91_MPU9250
#include <MPU9250.h>
#endif

#ifdef USE_GPS_NEO6M
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#endif

void dmpDataReady();

enum ROCKET_POSE { ROCKET_UNKNOWN, ROCKET_RISING, ROCKET_FALLING };

class IMU
{
private:
    bool used;
/* Sensors */
#ifdef USE_PERIPHERAL_MPU6050
    MPU6050 mpu;
#endif

/* Raw data */
#ifdef USE_PERIPHERAL_MPU6050
    // MPU control/status vars
    bool dmpReady = false;   // set true if DMP init was successful
    uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
    uint8_t devStatus;       // return status after each device operation (0 =
                             // success, !0 = error)
    uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;      // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];  // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;    // [w, x, y, z]         quaternion container
    VectorInt16 aa;  // [x, y, z]            accel sensor measurements
    VectorInt16
        aaReal;  // [x, y, z]            gravity-free accel sensor measurements

    VectorFloat gravity;  // [x, y, z]            gravity vector
    float euler[3];       // [psi, theta, phi]    Euler angle container
    float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and
    // gravity vector
#endif

    /* Filter */
    float altitude_filter(float v);

public:

#ifdef USE_PERIPHERAL_BMP280
    Adafruit_BMP280 bmp;  // I2C
    SimpleKalmanFilter altitudeKalmanFilter;
#endif

#ifdef USE_PERIPHERAL_MPU6050
    VectorInt16
        aaWorld;  // [x, y, z]            world-frame accel sensor measurements
#endif

#ifdef USE_GY91_MPU9250
    MPU9250 mpu;
#endif

#ifdef USE_GPS_NEO6M
    TinyGPSPlus gps;
    SoftwareSerial gpsSerial;
    String gpsCode;
#endif

    float altitude;  // Altitude
    float est_altitude;
    float velocity;

    float seaLevelHpa;

    ROCKET_POSE pose;

    /* IMU should be connected on the master I2C interface, this
     * function sets up all static parameters such as range
     * settings.
     */
    IMU();

    /* Perform calibration of sensors, including
     * 1. Magnetic sensor
     * 2. Gravitivity
     * 3. Sea level pressure
     */
    ERROR_CODE init();

/* Perform sensor update
 * 1. Acceleration for 1kHz
 * 2. Gyroscope for 1 kHz
 * 3. Magnetic for 100 Hz
 */
#if defined(USE_PERIPHERAL_MPU6050) || defined(USE_GY91_MPU9250)
    bool imu_isr_update();
#endif

    // Read pressure and temperature from bmp sensor
    float bmp_update();
};

#endif