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
    }
    logger.log_info(INFO_LOGGER_INIT);

    // Setup IMU
    while (imu.init() != IMU_OK) {
        logger.log_error(ERROR_IMU_INIT_FAILED);
        buzzer(BUZ_LEVEL0);
    }
    logger.log_info(INFO_IMU_INIT);

    // Servo position inialization
    parachute(SERVO_INITIAL_ANGLE);
    logger.log_info(INFO_SERVO_INIT);

    // Lora initialization
    logger.lora_init();
    logger.log_info(INFO_LORA_INIT);

    buzzer(BUZ_LEVEL3);

    // Setup core update
    logger.log_info(INFO_ALL_SYSTEM_INIT);

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