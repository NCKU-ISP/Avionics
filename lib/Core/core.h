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
#include <Ticker.h>
#include <WIFI_comms.h>
#include <logger.h>
#include <sensors.h>
#include "../../include/configs.h"


#include <ArduinoOTA.h>
#include <Servo.h>

#ifdef ENGINE_LOADING_TEST
#include <HX711.h>
#endif

#ifdef DE_SPIN_CONTROL
#include <PID_v1.h>
#endif

enum SYSTEM_STATE { SYSTEM_UP, SYSTEM_READY, SYSTEM_ERROR };
enum SPI_MASTER { SPI_NONE, SPI_SD, SPI_COMMUNICATION };
enum BUZZER_LEVEL { BUZ_NONE, BUZ_LEVEL1, BUZ_LEVEL2, BUZ_LEVEL3, BUZ_LEVEL4 };
enum CMD_TYPE { CMD_SERIAL, CMD_WIFI, CMD_BOTH };
enum ROCKET_STATE {
    ROCKET_READY,
    ROCKET_PREFLIGHT,
    ROCKET_OFFGROUND,
    ROCKET_LANDED
};
enum FAIRING_TYPE { F_SERVO, F_TRIGGER };
enum COMMS_STATE { WIFI_DISCONNECTED, WIFI_CONNECTED };
enum BOARD_TYPE { G_STATION, G_IGNITOR, O_AVIONICS};
typedef struct rocket_status {
    ROCKET_STATE state;
    bool fairingOpened;
    FAIRING_TYPE ftype;
    COMMS_STATE cState;
    BUZZER_LEVEL buzzState;
    bool liftoff;
    BOARD_TYPE btype;
} ROC_STATE;

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

    Ticker buzzer;
    Ticker fly_plan;
    Ticker log;
    Ticker stream;
    Ticker count_down;
    Ticker fire;
    Ticker pid_print;

    String data = "";

#ifdef PARACHUTE_SERVO
    Servo servo;
#elif defined(USE_SERVO_CONTROL)
    Servo servo[4];
#endif
    int closeAngle = SERVO_CLOSED_ANGLE,
        openAngle = SERVO_RELEASE_ANGLE;  // For setting servo angle
    bool wait_log = false;
    bool wait_stream = false;
    int release_t = RELEASE_TIME;
    int stop_t = STOP_TIME;
    int count_down_time = 10;

    void OTA_init();

#ifdef ENGINE_LOADING_TEST
    HX711 loadcell;
#endif

public:
    SYSTEM_STATE state;
    ROC_STATE rocket = {.state = ROCKET_READY,
                        .fairingOpened = false,
                        .ftype = F_SERVO,
                        .buzzState = BUZ_LEVEL1};
    Logger logger;
    SENSOR sensor;
#ifdef USE_WIFI_COMMUNICATION
    wifiServer comms;
#endif
    String core_cmd;

    System();

    /* Do all initialization task, including
     * 1. sensors
     * 2. logger
     */
    SYSTEM_STATE init(bool soft_init = true);

    /* For WiFi communication */
#ifdef DE_SPIN_CONTROL
    PID *reactionWheel;
    double gy_input, bldc_output, gy_target, bldc_init;
    double kp = 1, ki = 1, kd = 0;
    Servo bldc;
    bool PID_ON;
#endif

    void loop();


/* Check if the partner mcu report normal */
#ifdef USE_DUAL_SYSTEM_WATCHDOG
    WATCHDOG_STATE check_partner_state();
#endif

    BUZZER_LEVEL buzz(BUZZER_LEVEL beep, int times = 0);
    void trig(int pin, bool trig);
    void fairing(int angle);
    void servoOff();
    void setFairingLimit(int close, int open);
    void setServo(Servo *s, int angle);

    bool command(String cmd, CMD_TYPE type = CMD_SERIAL);
    void flight();
    void loading_test(String *command);

    void deSpinControl(bool ON);
};

#endif