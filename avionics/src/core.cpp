#include "core.h"
#include "Arduino.h"

#ifdef USE_DUAL_SYSTEM_WATCHDOG
SYSTEM_STATE System::state = SYSTEM_UP;
#endif

System::System() : imu(), logger()
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
    pinMode(PIN_BUZZER, OUTPUT);
    buzzer(LOW);
}

SYSTEM_STATE System::init()
{
    state = SYSTEM_ERROR;

    // Setup logger
    while (!logger.init())
        ;

    // Setup IMU
    while (imu.init() != IMU_OK)
        ;

    // Setup core update

    state = SYSTEM_READY;
    return state;
}

#ifdef USE_DUAL_SYSTEM_WATCHDOG
WATCHDOG_STATE System::check_partner_state()
{
    if (millis() - last_update_time > WATCH_SPI_TIMEOUT)
        return WATCHDOG_TIMEOUT;
    return WATCHDOG_OK;
}
#endif

void System::buzzer(bool beep)
{
    digitalWrite(PIN_BUZZER, beep);
}