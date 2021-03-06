/*
 * This library handle the preprocessing of IMU data, including
 * sensor initialization, Kalman filter and sensor fusion.
 *
 * Require:
 * Return:
 */

#ifndef _SENSOR_H
#define _SENSOR_H

#include "Arduino.h"
//#include "Wire.h"
#include <Adafruit_BMP280.h>
#include "configs.h"

#ifdef USE_PERIPHERAL_MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

void dmpDataReady();

enum IMU_STATE { IMU_RESET, IMU_OK, IMU_ERROR, IMU_INIT_ERROR };
enum ROCKET_POSE { ROCKET_UNKNOWN, ROCKET_RISING, ROCKET_FALLING };

class IMU
{
private:
    IMU_STATE state;

/* Sensors */
#ifdef USE_PERIPHERAL_MPU6050
    MPU6050 mpu;
#endif

#ifdef USE_PERIPHERAL_BMP280
    Adafruit_BMP280 bmp;  // I2C
#endif
// BMP280 bmp;

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
    float ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity
                   // vector
#endif

    /* Filter */
    float altitude_filter(float v);

public:
#ifdef USE_PERIPHERAL_MPU6050
    VectorInt16
        aaWorld;  // [x, y, z]            world-frame accel sensor measurements
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