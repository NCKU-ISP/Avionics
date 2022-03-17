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

#ifdef USE_LORA_COMMUNICATION
#include <../Lora/Lora.h>
#endif

#ifdef USE_PERIPHERAL_GPS
#include <NeoGps.h>
#endif
//#include <Servo.h>

enum SYSTEM_STATE { SYSTEM_UP = 0, SYSTEM_READY, SYSTEM_ERROR };
enum BUZZER_LEVEL { BUZ_NONE, BUZ_LEVEL0, BUZ_LEVEL1, BUZ_LEVEL2, BUZ_LEVEL3 };
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
    time_t last_bmp_update_time;
    time_t last_imu_update_time;
    time_t last_gps_update_time;
    time_t last_bat_update_time;
    BUZZER_LEVEL buz_state;
    time_t buz_start;

    // Servo servo;

public:
#ifdef USE_PERIPHERAL_GPS
    NeoGps gps;
#endif
    uint8_t battery_voltage;
    SYSTEM_STATE state;
    Logger logger;
    IMU imu;

    System();

    /* Do all initialization task, including
     * 1. sensors
     * 2. logger
     */
    SYSTEM_STATE init();
    bool buzzer(BUZZER_LEVEL beep);
    void buzzer_update();
    void trig(int pin, bool trig);
    void parachute(int angle);
    void parachute_release();
#ifdef USE_LORA_COMMUNICATION
    LoraPacketSystemState pack_system_state();
#endif
    void routine();
};

#endif