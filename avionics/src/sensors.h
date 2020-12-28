/*
 * This library handle the preprocessing of IMU data, including 
 * sensor initialization, Kalman filter and sensor fusion.
 * 
 * Require:
 * Return:
 */

#ifndef _SENSOR_H
#define _SENSOR_H

#include "configs.h"
#include "Arduino.h"
#include "Wire.h"
#include "MPU9250.h"
//#include "BMP280.h"
#include <Adafruit_BMP280.h>

enum IMU_STATE{IMU_RESET, IMU_OK, IMU_ERROR};

#ifdef USE_PERIPHERAL_MPU6050
typedef struct{
    float x;
    float y; 
    float z;
} Vector3f;
#endif

class IMU{
private:    
    IMU_STATE state;

    /* Sensors */
    #ifdef USE_PERIPHERAL_MPU6050
    static MPU9250 mpu; //(Wire, IMU_MPU_ADDR)
    #endif

    #ifdef USE_PERIPHERAL_BMP280
    Adafruit_BMP280 bmp; // I2C
    #endif
    //BMP280 bmp;

    /* Raw data */
    #ifdef USE_PERIPHERAL_MPU6050
    static Vector3f acc;    // Acceleration
    static Vector3f gyro;   // Gyroscope
    static Vector3f mag;    // Magnetometer
    #endif

    /* Filter */
    #ifdef USE_PERIPHERAL_MPU6050
    void acceleration_filter();

    void gyro_filter();

    void magnetic_filter();
    #endif

    float altitude_filter(float v);

public:
    float altitude; // Altitude

    float seaLevelHpa;
    
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
    IMU_STATE init();

    /* Perform sensor update 
     * 1. Acceleration for 1kHz
     * 2. Gyroscope for 1 kHz
     * 3. Magnetic for 100 Hz
     */
    #ifdef USE_PERIPHERAL_MPU6050
    static void imu_isr_update();
    #endif

    // Read pressure and temperature from bmp sensor
    void bmp_update();
};

#endif