#include "core.h"

System sys;

void setup()
{
    sys = System();
    while (sys.init() != SYSTEM_READY) {
        Serial.println("System start failed");
        sys.buzzer(BUZ_LEVEL2);
    }
    sys.logger.log("Init ok");
    ISR_enable();
}

void loop()
{
    if (sys.imu.pose == ROCKET_RISING) {
        sys.buzzer(BUZ_LEVEL0);
        sys.logger.log("Rising");
    } else if (sys.imu.pose == ROCKET_FALLING) {
        sys.parachute(SERVO_RELEASE_ANGLE);
        sys.buzzer(BUZ_LEVEL1);
        sys.logger.log("Falling");
    }

    // Continuous logging
    static auto last_log_time = millis();
    if (millis() - last_log_time > LOGGER_LOG_INTERVAL) {
        sys.logger.log(String(sys.imu.altitude));
        last_log_time = millis();
    }
}

void ISR_enable()
{
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B |= (1 << WGM22);  // CTC mode; Clear Timer on Compare
    TCCR2B = (TCCR2B & 0b11111000) | TIMER_PRESCALER_1024;
    // TCCR2B |= (1<<CS22);
    TIMSK2 |= (1 << OCIE2A);  // enable CTC for TIMER1_COMPA_vect
    TCNT2 = 0;                // counter 歸零
    OCR2A = 156;
    // 100 Hz
}

void ISR_disable()
{
    TCCR2A = 0;
    TCCR2B = 0;
}

ISR(TIMER2_COMPA_vect)
{
    sei();
    sys.imu.bmp_update();
    cli();
}