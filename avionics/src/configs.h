#ifndef _CONFIG_H
#define _CONFIG_H

/*--------------- System function enable ---------------*/
//#define USE_DUAL_SYSTEM_WATCHDOG
//#define USE_PERIPHERAL_SD_CARD
#define USE_PERIPHERAL_BMP280
//#define USE_PERIPHERAL_MPU6050
#define USE_SERIAL_DEBUGGER

/*--------------------- PIN_SETTING --------------------*/
#ifdef USE_DUAL_SYSTEM_WATCHDOG
#define PIN_PARTNER_RESET 8
#endif
#define PIN_SERVO

// Communication
#ifdef USE_DUAL_SYSTEM_WATCHDOG
#define PIN_SPI_CS_PARTNER 10
#endif

// Signal
#define PIN_BUZZER 2
#define PIN_TRIGGER 6
#define PIN_MOTOR 9

/*------------------ Constants for imu ------------------*/
#ifdef USE_PERIPHERAL_MPU6050
//#define USE_MPU_ISP_INTERFACE
// IMU interrupt
// Only available on specific pins,
// 328: 2, 3,
// 32u4: 0, 1, 2, 3, 7
#define PIN_IMU_INT 2
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
#define IMU_MPU_ADDR 0x68

#ifdef USE_MPU_ISP_INTERFACE
#define PIN_SPI_CS_IMU 9
#endif
#endif

// BMP280 setting
#ifdef USE_PERIPHERAL_BMP280
//#define IMU_BMP_ADDR       0x76
// The sampling times of sea level pressure while initializing
#define IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING 50
#define IMU_BMP_SAMPLING_PERIOD 10  // ms
#endif
// Altitude setting
// tau = (-T) / log(a), with a=0.8 and T=10(ms), tau about to 103.2 (ms)
#define IMU_ALTITUDE_SMOOTHING_CONSTANT 0.0f
#define IMU_RISING_CRITERIA 10.0f
#define IMU_FALLING_CRITERIA -10.0f

/*---------------------- Data logger --------------------*/
#ifdef USE_PERIPHERAL_SD_CARD
#define LOGGER_SD_CS 10
#define LOGGER_FILENAME "logger"
#define LOGGER_FILE_EXT ".txt"
#define LOGGER_FILENAME_BUFFER 20
#endif

/*-------------------- Serial debugger ------------------*/
#ifdef USE_SERIAL_DEBUGGER
#define SERIAL_DEBUGGER_BAUDRATE 9600
#endif

/*------------------ Watchdog protection ----------------*/
#ifdef USE_DUAL_SYSTEM_WATCHDOG
#define WATCH_SPI_TIMEOUT 100  // ms
#endif

#endif