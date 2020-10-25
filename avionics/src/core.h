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

#include "configs.h"
#include "Arduino.h"
#include "sensors.h"
#include "logger.h"

enum SYSTEM_STATE{SYSTEM_UP=0, SYSTEM_READY, SYSTEM_ERROR};
enum WATCHDOG_STATE{WATCHDOG_OK=0, WATCHDOG_TIMEOUT};

class System{
private:
    unsigned long last_update_time;
public:
    static SYSTEM_STATE state;
    IMU imu;
    Logger logger;

    System();

    /* Do all initialization task, including
     * 1. sensors
     * 2. logger
     */
    SYSTEM_STATE init();

    /* Check if the partner mcu report normal */
    WATCHDOG_STATE check_partner_state();

    void buzzer(bool beep);
};

#endif