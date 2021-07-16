#ifndef _CONFIG_H
#define _CONFIG_H

#include "Arduino.h"
/*------------------------------------------------------*/
/*--------------- System function enable ---------------*/
/*------------------------------------------------------*/
//#define USE_DUAL_SYSTEM_WATCHDOG
#define USE_PERIPHERAL_SD_CARD
#define USE_PERIPHERAL_BMP280
//#define USE_PERIPHERAL_BMP280_LIB
#define USE_PERIPHERAL_MPU6050
#define USE_PERIPHERAL_BUZZER
#define USE_SERIAL_DEBUGGER
#define USE_LORA_COMMUNICATION
#define USE_PERIPHERAL_GPS

/*------------------------------------------------------*/
/*--------------------- PIN_SETTING --------------------*/
/*------------------------------------------------------*/

/* Pin setting with ESP32 */
#ifdef ARDUINO_ARCH_ESP32

// Output devices

#ifdef USE_PERIPHERAL_BUZZER
#define PIN_BUZZER 12
#endif

// Signals
#define PIN_TRIGGER_A 10
#define PIN_TRIGGER_B 11

#define PIN_SERVO_A 27
#define PIN_SERVO_B 32
#define PIN_SERVO_C 33
#define PIN_SERVO_D 13
#define PIN_SERVO_E 14

// Input devices
#define PIN_VOLTAGE_DETECT 4

// Communication
#ifdef USE_LORA_COMMUNICATION
#define PIN_SLORA_INTERRUPT 35  // DIO1
#define PIN_SLORA_BUSY 34       // BUSY
#define PIN_SLORA_RESET 2       // RST
#define PIN_SLORA_SELECT 15     // NSS
#define PIN_SLORA_SCLK 18
#define PIN_SLORA_MISO 19
#define PIN_SLORA_MOSI 23
#define PIN_RADIO_TXEN -1
#define PIN_RADIO_RXEN -1
#endif

#ifdef USE_PERIPHERAL_GPS
#endif

#ifdef USE_PERIPHERAL_SD_CARD
#define LOGGER_SD_CS 5
#endif

/* Pin setting with ESP8266 */
#elif defined(ARDUINO_ARCH_ESP8266)
#define PIN_SERVO

/* Pin setting with Atmega328p */
#elif defined(ARDUINO_AVR_UNO)
// Pin mapping of atmega328p
// A0~A5 map to 14~19
// A0-14, A1-15, A2-16, A3-17, A4-18, A5-19

#endif
#define PIN_TRIGGER PIN_TRIGGER_A

/*------------------------------------------------------*/
/*-------------------- Communication -------------------*/
/*------------------------------------------------------*/
#ifdef USE_LORA_COMMUNICATION
#define RF_FREQUENCY 868000000  // Hz center frequency
// bandwidth=125khz  0:250kHZ,1:125kHZ,2:62kHZ,3:20kHZ....
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define TX_OUTPUT_POWER 22  // dBm tx output power
#define LORA_CODINGRATE 1
// [1: 4/5,     \
//  2: 4/6,     \
//  3: 4/7,     \
//  4: 4/8]

#define LORA_PREAMBLE_LENGTH 8            // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0             // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false  // variable data payload
#define LORA_IQ_INVERSION_ON false
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000
#define LORA_PACKET_SIZE 64  // Define the payload size here
#endif

#ifdef USE_DUAL_SYSTEM_WATCHDOG
#define PIN_SPI_CS_PARTNER 10
#define PIN_PARTNER_RESET 8
#endif

/*------------------------------------------------------*/
/*------------ Configuration for parachute -------------*/
/*------------------------------------------------------*/
#define SERVO_INITIAL_ANGLE 180
#define SERVO_RELEASE_ANGLE 0

/*------------------------------------------------------*/
/*------------------ Constants for imu -----------------*/
/*------------------------------------------------------*/
#ifdef USE_PERIPHERAL_MPU6050
//#define USE_MPU_ISP_INTERFACE
// IMU interrupt
// Only available on specific pins,
// 328: 2, 3,
// 32u4: 0, 1, 2, 3, 7
//#define PIN_IMU_INT 2
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
#define IMU_MPU_ADDR 0x68
#define IMU_ISR_SAMPLING_PERIOD 10  // ms
#endif

/*------------------------------------------------------*/
/*------------------------- GPS ------------------------*/
/*------------------------------------------------------*/
#ifdef USE_PERIPHERAL_GPS
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17

// once you modify this, you should reconfigurate your GPS module.
#define GPS_DEFAULT_BAUDRATE 9600
#define GPS_SAMPLING_PERIOD 200  // ms
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
#define IMU_ALTITUDE_SMOOTHING_CONSTANT 0.8f
#define IMU_RISING_CRITERIA 10.0f
#define IMU_FALLING_CRITERIA -10.0f
#define IMU_ACCEL_CRITERIA_MAGNITUDE 5
#define IMU_ACCEL_CRITERIA_INNER 0.7f

/*------------------------------------------------------*/
/*---------------------- Data logger -------------------*/
/*------------------------------------------------------*/
#ifdef USE_PERIPHERAL_SD_CARD
#define LOGGER_FILENAME "/logger"
#define LOGGER_FILE_EXT ".txt"
#define LOGGER_FILENAME_BUFFER 20
#endif
#define LOGGER_LOG_INTERVAL 50

/*------------------------------------------------------*/
/*-------------------- Serial debugger -----------------*/
/*------------------------------------------------------*/
#ifdef USE_SERIAL_DEBUGGER
#define SERIAL_DEBUGGER_BAUDRATE 115200
#endif

/*------------------------------------------------------*/
/*------------------ Watchdog protection ---------------*/
/*------------------------------------------------------*/
#ifdef USE_DUAL_SYSTEM_WATCHDOG
#define WATCH_SPI_TIMEOUT 100  // ms
#endif

/*------------------------------------------------------*/
/*-------------- Battery Voltage Detector --------------*/
/*------------------------------------------------------*/
#define PIN_BATTERY_VOLTAGE_DETECTOR 36
#define BATTERY_VOLATGE_LOWER_BOUND 4
#define BATTERY_VOLATGE_UPPER_BOUND 8
#define BATTERY_CIRCUIT_SCALING 57 / 10
#define BATTERY_SAMPLING_PERIOD 500  // ms

enum ERROR_CODE {
    ERROR_OK,
    ERROR_LOGGER_INIT_FAILED,
    ERROR_SD_INIT_FAILED,
    ERROR_MPU_INIT_FAILED,
    ERROR_DMP_INIT_FAILED,
    ERROR_BMP_INIT_FAILED,
    ERROR_IMU_INIT_FAILED
};

enum INFO_CODE {
    INFO_LOGGER_INIT,
    INFO_IMU_INIT,
    INFO_SERVO_INIT,
    INFO_ALL_SYSTEM_INIT,
    INFO_LORA_INIT,
    INFO_RISING,
    INFO_FALLING
};

#endif