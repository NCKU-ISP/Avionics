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

#include <../../include/configs.h>
#include <../Logger/Logger.h>
#include <../Sensors/Sensors.h>
#include <Arduino.h>

//#include <Servo.h>

enum SYSTEM_STATE { SYSTEM_UP = 0, SYSTEM_READY, SYSTEM_ERROR };
enum BUZZER_LEVEL { BUZ_LEVEL0, BUZ_LEVEL1, BUZ_LEVEL2, BUZ_LEVEL3 };
/*
#define TIMER_PRESCALER_1 0x01
#define TIMER_PRESCALER_8 0x02
#define TIMER_PRESCALER_32 0x03
#define TIMER_PRESCALER_64 0x04
#define TIMER_PRESCALER_128 0x05
#define TIMER_PRESCALER_256 0x06
#define TIMER_PRESCALER_1024 0x07
*/
class System
{
private:
    unsigned long last_update_time;
    // volatile SPI_MASTER sd_master;

    // Servo servo;

public:
    SYSTEM_STATE state;
    Logger logger;
    IMU imu;

    System();

    /* Do all initialization task, including
     * 1. sensors
     * 2. logger
     */
    SYSTEM_STATE init();
    void buzzer(BUZZER_LEVEL beep);
    void trig(int pin, bool trig);
    void parachute(int angle);
    void parachute_release();
};

#endif