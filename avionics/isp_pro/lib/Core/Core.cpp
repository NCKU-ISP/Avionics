#include "Core.h"

System::System() : logger(), imu()
{
    Serial.begin(115200);
// Signal
#ifdef USE_PERIPHERAL_BUZZER
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    buz_state = BUZ_NONE;
#endif

// Trigger
#ifdef PIN_TRIGGER_A
    pinMode(PIN_TRIGGER_A, OUTPUT);
    trig(PIN_TRIGGER_A, LOW);
#endif
#ifdef PIN_TRIGGER_B
    pinMode(PIN_TRIGGER_B, OUTPUT);
    trig(PIN_TRIGGER_B, LOW);
#endif

    last_bmp_update_time = millis();
    last_imu_update_time = millis();
    last_gps_update_time = millis();
    last_bat_update_time = millis();
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
        buzzer(BUZ_NONE);
    }
    // logger.log_info(INFO_IMU_INIT);
    logger.log_code(INFO_IMU_INIT, LEVEL_INFO);

    // Servo position inialization
    parachute(SERVO_INITIAL_ANGLE);
    logger.log_code(INFO_SERVO_INIT, LEVEL_INFO);

    // Lora initialization

    // Gps initialization
#ifdef USE_PERIPHERAL_GPS
    gps.begin();
#endif

    buzzer(BUZ_LEVEL3);

    // Setup core update
    logger.log_code(INFO_ALL_SYSTEM_INIT, LEVEL_INFO);
    return SYSTEM_READY;
}

void System::buzzer_update()
{
#ifdef USE_PERIPHERAL_BUZZER
    const int time_quantum = 50;
    const time_t delta = (millis() - buz_start) / time_quantum;
    bool beep;
    switch (buz_state) {
    case BUZ_LEVEL0:
        beep = delta % 2 == 0;
        digitalWrite(PIN_BUZZER, beep);
        if (delta >= 2 - 1)
            buz_state = BUZ_NONE;
        break;
    case BUZ_LEVEL1:
        beep = delta % 2 == 0;
        digitalWrite(PIN_BUZZER, beep);
        if (delta >= 6 - 1)
            buz_state = BUZ_NONE;
        break;
    case BUZ_LEVEL2:
        beep = (delta % 2 == 0) || (delta < 7);
        digitalWrite(PIN_BUZZER, beep);
        if (delta >= 10 - 1)
            buz_state = BUZ_NONE;
        break;
    case BUZ_LEVEL3:
        beep = (delta % 2 == 0) || (delta < 7);
        digitalWrite(PIN_BUZZER, beep);
        if (delta >= 12 - 1)
            buz_state = BUZ_NONE;
        break;

    default:
        break;
    }
#endif
}

bool System::buzzer(BUZZER_LEVEL beep)
{
    if (buz_state == BUZ_NONE) {
        buz_state = beep;
        buz_start = millis();
        return true;
    }
    return false;
}

void System::trig(int pin, bool trig)
{
    digitalWrite(pin, trig);
}

void System::parachute(int angle)
{
    trig(PIN_TRIGGER, true);
    // servo.attach(PIN_MOTOR);
    // servo.write(angle);
}

void System::parachute_release()
{
    // release the servo to save power
    // servo.detach();
    trig(PIN_TRIGGER, false);
}

#ifdef USE_LORA_COMMUNICATION
LoraPacketSystemState System::pack_system_state()
{
    // Gps
#ifndef USE_PERIPHERAL_GPS
    const int8_t num_satellite = 0;
    const int32_t longitude = 0;
    const int32_t latitude = 0;
#else
    const int8_t num_satellite = gps.number_of_satellites;
    const int32_t longitude = gps.longitude;
    const int32_t latitude = gps.latitude;
#endif

    // IMU
    const float altitude = imu.altitude;
    const float speed = 0.0;
    const float rolling = 0.0;

    return LoraPacketSystemState(0, battery_voltage, num_satellite, altitude,
                                 speed, rolling, longitude, latitude);
}
#endif

void System::routine()
{
#ifdef USE_PERIPHERAL_BMP280
    if (last_bmp_update_time - millis() > IMU_BMP_SAMPLING_PERIOD) {
        imu.bmp_update();
        last_bmp_update_time = millis();
    }
#endif

    if (last_imu_update_time - millis() > IMU_ISR_SAMPLING_PERIOD) {
        imu.imu_isr_update();
        last_imu_update_time = millis();
    }

#ifdef USE_PERIPHERAL_GPS
    if (last_gps_update_time - millis() > GPS_SAMPLING_PERIOD) {
        gps.update();
        last_gps_update_time = millis();
    }
#endif

    if (last_bat_update_time - millis() > BATTERY_SAMPLING_PERIOD) {
        // Battery voltage detector
        const uint32_t voltage_raw = analogRead(PIN_BATTERY_VOLTAGE_DETECTOR) *
                                     BATTERY_CIRCUIT_SCALING / 10 * 33;
        // rescale to 1 byte, range from lower bound to upper bound
        battery_voltage =
            (voltage_raw - BATTERY_VOLATGE_LOWER_BOUND * 4096) /
            ((BATTERY_VOLATGE_UPPER_BOUND - BATTERY_VOLATGE_LOWER_BOUND) *
             (4096 / 256));
        last_bat_update_time = millis();
    }

    buzzer_update();
}