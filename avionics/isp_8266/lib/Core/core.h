/*
 * This library deals with all system protection mechanism.
 * Including
 * 1. Watchdog protection
 * 2. System state protection
 * 3. Error handler
 * Require:
 * Return:
 */
#ifndef _CORE_H
#define _CORE_H

#include <Arduino.h>
#include <../../include/configs.h>
#include <logger.h>
#include <sensors.h>
#include <WIFI_comms.h>


#include <ArduinoOTA.h>
#include <Servo.h>

#ifdef ENGINE_LOADING_TEST
#include <HX711.h>
#endif

enum SYSTEM_STATE { SYSTEM_UP = 0, SYSTEM_READY, SYSTEM_ERROR };
enum SPI_MASTER { SPI_NONE, SPI_SD, SPI_COMMUNICATION};
enum BUZZER_LEVEL { BUZ_LEVEL0, BUZ_LEVEL1, BUZ_LEVEL2, BUZ_LEVEL3 };

#define TIMER_PRESCALER_1 0x01
#define TIMER_PRESCALER_8 0x02
#define TIMER_PRESCALER_32 0x03
#define TIMER_PRESCALER_64 0x04
#define TIMER_PRESCALER_128 0x05
#define TIMER_PRESCALER_256 0x06
#define TIMER_PRESCALER_1024 0x07

#ifdef USE_DUAL_SYSTEM_WATCHDOG
enum WATCHDOG_STATE { WATCHDOG_OK = 0, WATCHDOG_TIMEOUT };
#endif

class System
{
private:
    unsigned long last_update_time;
    volatile SPI_MASTER sd_master;

    Servo servo;
    int closeAngle = SERVO_INITIAL_ANGLE
      , openAngle = SERVO_RELEASE_ANGLE;    // For setting servo angle
    bool start = false;
    int release_t = RELEASE_TIME;
    int stop_t = STOP_TIME;

    void OTA_init();

    #ifdef ENGINE_LOADING_TEST
    HX711 loadcell;
    #endif

public:
    SYSTEM_STATE state;
    IMU imu;
    Logger logger;
#ifdef USE_WIFI_COMMUNICATION
    wifiServer comms;
#endif

    System();

    /* Do all initialization task, including
     * 1. sensors
     * 2. logger
     */
    SYSTEM_STATE init();

/* For WiFi communication */
    bool wifi_send(uint8_t num, String payload);
    bool wifi_send(uint8_t num, const char * payload);

    bool wifi_broadcast(String payload);
    bool wifi_broadcast(const char * payload);

    void loop();


/* Check if the partner mcu report normal */
#ifdef USE_DUAL_SYSTEM_WATCHDOG
    WATCHDOG_STATE check_partner_state();
#endif

    void buzzer(BUZZER_LEVEL beep);
    void trig(bool trig);
    void fairingOpen(int angle = SERVO_RELEASE_ANGLE);
    void fairingClose(int angle = SERVO_INITIAL_ANGLE);
    void fairingServoOff();
    void setFairingLimit(int close, int open);
    void setMotor(int motor, int angle);

    void command(String *command);
    void flight();
    void loading_test(String *command);
};

#endif