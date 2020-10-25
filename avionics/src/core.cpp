#include "core.h"
#include "Arduino.h"

#include "configs.h"
#include "sensors.h"
#include "logger.h"

System::System():imu(), logger(){
    // Pin set up
    pinMode(PIN_PARTNER_RESET, OUTPUT);
    digitalWrite(PIN_PARTNER_RESET, LOW);

    // Chip select pin for partner mcu
    pinMode(PIN_SPI_CS_PARTNER, OUTPUT);
    digitalWrite(PIN_SPI_CS_PARTNER, HIGH);
    
    // Chip select pin for MPU9250
    pinMode(PIN_SPI_CS_IMU, OUTPUT);
    digitalWrite(PIN_SPI_CS_IMU, HIGH);

    // Signal
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}

SYSTEM_STATE System::init(){
    state = SYSTEM_ERROR;
    if ( !logger.init() ) return state;
    if ( imu.init() != IMU_OK ) return state;
    state = SYSTEM_READY;
    return state;
}

WATCHDOG_STATE System::check_partner_state(){
    if ( millis() - last_update_time > WATCH_SPI_TIMEOUT )
        return WATCHDOG_TIMEOUT;
    return WATCHDOG_OK;
}

void System::buzzer(bool beep){
    digitalWrite(PIN_BUZZER, beep);
}