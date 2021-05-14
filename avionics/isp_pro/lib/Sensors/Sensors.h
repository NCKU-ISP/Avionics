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
#include <Bmp280.h>
#endif

#ifdef USE_PERIPHERAL_MPU6050
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#endif


void dmpDataReady();

#define IMU_DATA_READY_SD 0x01
#define IMU_DATA_READY_LORA 0x02

enum ROCKET_POSE { ROCKET_UNKNOWN, ROCKET_RISING, ROCKET_FALLING };

class IMU
{
private:
/* Sensors */
#ifdef USE_PERIPHERAL_MPU6050
    MPU6050 mpu;
#endif

#ifdef USE_PERIPHERAL_BMP280
    Adafruit_BMP280 bmp;  // I2C
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

#endif

    /* Filter */
    float altitude_filter(float v);

public:
    uint8_t imu_update_flag;
#ifdef USE_PERIPHERAL_MPU6050
    // [x, y, z]            world-frame accel sensor measurements
    VectorInt16 aaWorld;
    uint16_t mpu_last_update_time;
    Quaternion q;  // [w, x, y, z]         quaternion container
#endif
    float altitude;  // Altitude

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
#ifdef USE_PERIPHERAL_MPU6050
    bool imu_isr_update();
#endif

#ifdef USE_PERIPHERAL_BMP280
    // Read pressure and temperature from bmp sensor
    void bmp_update();
#endif
};

#endif