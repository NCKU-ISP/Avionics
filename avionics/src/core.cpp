#include "core.h"
#include "Arduino.h"

#ifdef USE_DUAL_SYSTEM_WATCHDOG
SYSTEM_STATE System::state = SYSTEM_UP;
#endif

System::System() : logger(), imu()
{
// Pin set up
#ifdef USE_DUAL_SYSTEM_WATCHDOG
    pinMode(PIN_PARTNER_RESET, OUTPUT);
    digitalWrite(PIN_PARTNER_RESET, LOW);

    // Chip select pin for partner mcu
    pinMode(PIN_SPI_CS_PARTNER, OUTPUT);
    digitalWrite(PIN_SPI_CS_PARTNER, HIGH);
#endif

// Signal
#ifdef USE_PERIPHERAL_BUZZER
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
#endif

    // Trigger
    pinMode(PIN_TRIGGER, OUTPUT);
    trig(LOW);
}

SYSTEM_STATE System::init()
{
    // Setup logger
    while (!logger.init()) {
        buzzer(BUZ_LEVEL0);
        logger.log_code(ERROR_LOGGER_INIT_FAILED, LEVEL_ERROR);
    }
    logger.log_code(INFO_LOGGER_INIT, LEVEL_INFO);

    // Setup IMU
    ERROR_CODE err = ERROR_MPU_INIT_FAILED;
    while (err != ERROR_OK) {
        err = imu.init();
        logger.log_code(err, LEVEL_ERROR);
        buzzer(BUZ_LEVEL0);
    }
    // logger.log_info(INFO_IMU_INIT);
    logger.log_code(INFO_IMU_INIT, LEVEL_INFO);

    // Servo position inialization
    parachute(SERVO_INITIAL_ANGLE);
    logger.log_code(INFO_SERVO_INIT, LEVEL_INFO);

    // Lora initialization
    logger.lora_init();
    logger.log_code(INFO_LORA_INIT, LEVEL_INFO);

    buzzer(BUZ_LEVEL3);

    // Setup core update
    logger.log_code(INFO_ALL_SYSTEM_INIT, LEVEL_INFO);

    return SYSTEM_READY;
}

#ifdef USE_DUAL_SYSTEM_WATCHDOG
WATCHDOG_STATE System::check_partner_state()
{
    if (millis() - last_update_time > WATCH_SPI_TIMEOUT)
        return WATCHDOG_TIMEOUT;
    return WATCHDOG_OK;
}
#endif

void System::buzzer(BUZZER_LEVEL beep)
{
#ifdef USE_PERIPHERAL_BUZZER
    const int time_quantum = 100;
    switch (beep) {
    case BUZ_LEVEL0:
        for (int i = 0; i < 2; i++) {
            digitalWrite(PIN_BUZZER, i & 0x1);
            delay(time_quantum);
        }
        digitalWrite(PIN_BUZZER, LOW);
        break;
    case BUZ_LEVEL1:
        for (int i = 0; i < 2; i++) {
            digitalWrite(PIN_BUZZER, i & 0x1);
            delay(time_quantum * 5);
        }
        digitalWrite(PIN_BUZZER, LOW);
        break;
    case BUZ_LEVEL2:
        for (int i = 0; i <= 5; i++)
            buzzer(BUZ_LEVEL0);
        break;
    case BUZ_LEVEL3:
        buzzer(BUZ_LEVEL1);
        for (int i = 0; i < 2; i++)
            buzzer(BUZ_LEVEL0);
        break;

    default:
        digitalWrite(PIN_BUZZER, LOW);
        break;
    }
#endif
}

void System::trig(bool trig)
{
    digitalWrite(PIN_TRIGGER, trig);
}

void System::parachute(int angle)
{
    trig(true);
    servo.attach(PIN_MOTOR);
    servo.write(angle);
}

void System::parachute_release()
{
    // release the servo to save power
    servo.detach();
    trig(false);
}