#ifndef _CONFIG_H
#define _CONFIG_H

/* PIN_SETTING */
#define PIN_PARTNER_RESET  8

// Communication
#define PIN_SPI_CS_PARTNER 10
#define PIN_SPI_CS_IMU     9

// Signal
#define PIN_BUZZER         4

// IMU interrupt
// Only available on specific pins, 
// 328: 2, 3, 
// 32u4: 0, 1, 2, 3, 7
#define PIN_IMU_INT        2

/* Constants for imu */
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
#define IMU_MPU_ADDR       0x68
//#define IMU_BMP_ADDR       0x76

#define IMU_BMP_OVERSAMPLE 4

/* Data logger */
#define LOGGER_SD_CS        10
#define LOGGER_FILENAME "logger"
#define LOGGER_FILE_EXT ".txt"
#define LOGGER_FILENAME_BUFFER 20

/* Watchdog protection */
#define WATCH_SPI_TIMEOUT 100   // ms

#endif