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

#include "Arduino.h"
#include "configs.h"
#include "logger.h"
#include "sensors.h"

#include <Servo.h>

enum SYSTEM_STATE { SYSTEM_UP = 0, SYSTEM_READY, SYSTEM_ERROR };
enum SPI_MASTER { SPI_NONE, SPI_SD, SPI_COMMUNICATION };
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

public:
    SYSTEM_STATE state;
    IMU imu;
    Logger logger;

    System();

    /* Do all initialization task, including
     * 1. sensors
     * 2. logger
     */
    SYSTEM_STATE init();

/* Check if the partner mcu report normal */
#ifdef USE_DUAL_SYSTEM_WATCHDOG
    WATCHDOG_STATE check_partner_state();
#endif

    void buzzer(BUZZER_LEVEL beep);
    void trig(bool trig);
    void parachute(int angle);
};

#endif