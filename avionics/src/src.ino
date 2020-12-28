#include "core.h"

System sys = System();

void setup(){
    Serial.begin(9600);
    while ( sys.init() != SYSTEM_READY ){
        sys.buzzer(true);
        delay(500);
        sys.buzzer(false);
        delay(500);
    }
    Serial.println("Init ok");
    //ISR_enable();
}

void loop(){
    sys.imu.bmp_update();
    Serial.println(sys.imu.altitude);
    delay(100);
}

void ISR_enable(){
    TCCR2A = 0;
    TCCR2B = 0; 
    TCCR2B |= (1<<WGM22);  // CTC mode; Clear Timer on Compare
    TCCR2B = (TCCR2B & 0b11111000) | TIMER_PRESCALER_64;
    //TCCR2B |= (1<<CS22);
    TIMSK2 |= (1 << OCIE2A);  // enable CTC for TIMER1_COMPA_vect
    TCNT2=0;  // counter 歸零 
    OCR2A = 2500;
    // 100 Hz
}

void ISR_disable(){
    TCCR2A = 0;
    TCCR2B = 0; 
}

ISR(TIMER2_COMPA_vect){
    sys.imu.bmp_update();
    Serial.println(sys.imu.altitude);
}