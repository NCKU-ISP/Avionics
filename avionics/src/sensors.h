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
#include "BMP280.h"

enum IMU_STATE{IMU_RESET, IMU_OK, IMU_ERROR};

typedef struct{
    float x;
    float y; 
    float z;
} Vector3f;

class IMU{
private:    
    static IMU_STATE state;

    /* Sensors */
    static MPU9250 mpu; //(Wire, IMU_MPU_ADDR)
    static BMP280 bmp;

    /* Raw data */
    static Vector3f acc;    // Acceleration
    static Vector3f gyro;   // Gyroscope
    static Vector3f mag;    // Magnetometer
    static double temper;   // Temperature
    static double pressure; // Pressure
    static double altitude; // Altitude

    static double sea_level_pressure;

    /* Filter */
    static void acceleration_filter();

    static void gyro_filter();

    static double pressure_filter(double pressure);

    static void magnetic_filter();

public:
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
    static IMU_STATE init();

    /* Perform sensor update 
     * 1. Acceleration for 1kHz
     * 2. Gyroscope for 1 kHz
     * 3. Magnetic for 100 Hz
     */
    static void imu_isr_update();

    // Read pressure and temperature from bmp sensor
    static void bmp_update();
};

#endif