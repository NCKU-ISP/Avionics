#include "core.h"

System *sys;

bool run_mpu = false;
bool test_connection = false;
bool a = false;
bool b = false;
bool record = false;
unsigned long release_time;
int timing = RELEASE_TIME;

void setup()
{
    Serial.begin(115200);
    // Serial.println();
    // Serial.println("hello");
    // // Serial.setDebugOutput(false);
    // Serial.end();
    sys = new System();

    while (sys->init() != SYSTEM_READY) {
        sys->buzzer(BUZ_LEVEL2);
    }
    test_connection = false;
    // ISR_enable();
    // pinMode(0, OUTPUT);
    // pinMode(2, OUTPUT);
    // pinMode(15, OUTPUT);
    // pinMode(13, OUTPUT);
}

void loop()
{
    sys->loop();
    // sys->imu.imu_isr_update();
    while (Serial.available()) {
        static String command = "";
        char c = (char) Serial.read();
        if (c != '\n')
            command += c;
        else {
            if (command.equals("list file"))
                sys->logger.listFile();
            else if (command == "format")
                sys->logger.formatFS();
            else if (command.indexOf("delete") != -1)
                sys->logger.deleteFile(command.substring(7));
            else if (command.indexOf("read") != -1){
                static int pos = 0;
                while(pos != -1)
                    Serial.print(sys->logger.readFile(command.substring(5), &pos));
                pos = 0;
            }
            else if (command == "calibrate")
                sys->imu.init();
            else {
                Serial.println(command);
                Serial.println("command not found");
            }
            command = "";
        }
    }
    if(sys->comms.message == "init"){
        sys->fairingClose(SERVO_INITIAL_ANGLE);
        sys->comms.message = "";
    } else if(sys->comms.message == "open"){
        sys->fairingOpen(SERVO_RELEASE_ANGLE);
        sys->comms.message = "";
    } else if(sys->comms.message == "detach"){
        sys->fairingServoOff();
        sys->comms.message = "";
    }
    // if (sys->comms.message.equals("Sing")) {
    //     sys->comms.message = "";
    //     digitalWrite(0, 1);
    //     digitalWrite(2, 1);
    //     digitalWrite(15, 0);
    //     tone(13, 261, 500);
    //     delay(500);
    //     tone(13, 293, 500);
    //     delay(500);
    //     tone(13, 329, 500);
    //     delay(500);
    //     tone(13, 349, 500);
    //     delay(500);
    //     tone(13, 392, 500);
    //     delay(500);
    //     tone(13, 440, 500);
    //     delay(500);
    //     tone(13, 493, 500);
    //     delay(500);
    //     tone(13, 523, 500);
    //     delay(500);
    //     digitalWrite(0, 0);
    //     digitalWrite(2, 0);
    //     digitalWrite(15, 0);
    // } else if (sys->comms.message.equals("T0")) {
    //     sys->comms.message = "";
    //     sys->setMotor(0, 100);
    // } else if (sys->comms.message.equals("T1")) {
    //     sys->comms.message = "";
    //     sys->setMotor(1, 200);
    // } else if (sys->comms.message.equals("Temp")) {
    //     sys->comms.message = "";
    //     float p = sys->imu.bmp.readPressure();
    //     sys->wifi_broadcast(String("height: ") + String(sys->imu.bmp.readAltitude(p)));
    //     sys->wifi_broadcast(String("pressure: ") + String(p));
    //     sys->wifi_broadcast(String("temp: ") + String(sys->imu.bmp.readTemperature()));
    // } else if (sys->comms.message.equals("Trig")) {
    //     sys->comms.message = "";
    //     sys->trig(true);
    //     delay(1000);
    //     sys->trig(false);
    // }

    // if (sys->comms.message.equals("stop mpu")) {
    //     run_mpu = false;
    //     sys->comms.message = "";
    // } else if (sys->comms.message.equals("run mpu")) {
    //     run_mpu = true;
    //     sys->comms.message = "";
    // } else if (sys->comms.message.equals("run test")) {
    //     test_connection = true;
    //     sys->comms.message = "";
    // } else if (sys->comms.message.equals("stop test")) {
    //     test_connection = false;
    //     sys->comms.message = "";
    // } else if (sys->comms.message.equals("reset")) {
    //     sys->comms.message = "";
    //     ESP.restart();
    //     ESP.reset();
    // } else if (sys->comms.message.equals("record")) {
    //     record = true;
    //     sys->comms.message = "";
    // } else if (sys->comms.message.equals("stop record")) {
    //     record = false;
    //     b = true;
    //     sys->comms.message = "";
    // } else if (sys->comms.message.equals("launch")) {
    //     sys->wifi_broadcast("launch");
    //     sys->comms.message = "";
    // } else if (sys->comms.message.equals("lock")) {
    //     sys->wifi_broadcast("lock");
    //     sys->comms.message = "";
    // } else if (sys->comms.message.substring(0, 4) == "time") {
    //     sys->comms.message.remove(0, 5);
    //     timing = sys->comms.message.toInt();
    //     sys->wifi_broadcast(String("Set release time to ") + timing + "ms");
    //     sys->comms.message = "";
    // }

    // if (test_connection) {
    //     static auto time = millis();
    //     if (millis() - time > 200) {
    //         time = millis();
    //         if (a) {
    //             sys->wifi_broadcast("1");
    //             a = false;
    //         } else {
    //             sys->wifi_broadcast("0");
    //             a = true;
    //         }
    //     }
    // }
    // if ((abs(sys->imu.mpu.getAccelX_mss()) > 50 ||
    //     abs(sys->imu.mpu.getAccelY_mss()) > 50 ||
    //     abs(sys->imu.mpu.getAccelZ_mss()) > 50) && record == false) {
    //     record = true;
    //     Serial.println("start recording");
    //     sys->wifi_broadcast("start recording");
    // }
    // if(record == false) release_time = millis();
    // if (sys->imu.pose == ROCKET_RISING) {
    //     // sys->buzzer(BUZ_LEVEL0);
    //     sys->wifi_broadcast("Rising");
    //     sys->logger.log("Rising");
    // }
    // if (sys->imu.pose == ROCKET_FALLING) {
    //     sys->fairingOpen();
    //     sys->wifi_broadcast("Falling");
    //     // sys->buzzer(BUZ_LEVEL1);
    //     sys->logger.log("Falling");
    // }
    // #ifdef release_by_time
    //     if(millis() - release_time > timing && a == false && b == false){
    //         sys->fairingOpen();
    //         sys->wifi_broadcast("release");
    //         sys->logger.log("release");
    //         a = true;
    //     }
    // #endif

    // // Continuous logging
    // static auto last_log_time = millis();
    // if (millis() - last_log_time > 50 && (run_mpu || record)) {
    //     sys->imu.bmp_update();
    //     sys->logger.log(String(sys->imu.altitude), LEVEL_INFO);
    //     last_log_time = millis();


    // #if defined(USE_PERIPHERAL_MPU6050) || defined(USE_GY91_MPU9250)
    //         if (sys->imu.imu_isr_update()) {
    //             // int16_t data[3] = {sys->imu.aaWorld.x, sys->imu.aaWorld.y,
    //             //                    sys->imu.aaWorld.z};
    //             float data[10] = {
    //                 sys->imu.mpu.getAccelX_mss(),
    //                 sys->imu.mpu.getAccelY_mss(),
    //                 sys->imu.mpu.getAccelZ_mss(),
    //                 sys->imu.mpu.getGyroX_rads(),
    //                 sys->imu.mpu.getGyroY_rads(),
    //                 sys->imu.mpu.getGyroZ_rads(),
    //                 sys->imu.mpu.getMagX_uT(),    sys->imu.mpu.getMagY_uT(),
    //                 sys->imu.mpu.getMagZ_uT(),
    //                 sys->imu.mpu.getTemperature_C()};
    // #ifdef USE_LORA_COMMUNICATION
    //             sys->logger.lora_send(LORA_ACCEL, data);
    // #elif defined(USE_GY91_MPU9250)
    //             String payload = String(data[0]);
    //             for (int i = 1; i < 10; i++) {
    //                 payload += "\t" + String(data[i]);
    //             }
    //             sys->wifi_broadcast(payload);
    //             Serial.println(payload);
    // #endif
    //         }
    // #endif
    // }
    // String gps_text = "";
    // while(sys->imu.gpsSerial.available()){
    //     char m = sys->imu.gpsSerial.read();
    //     if(m != '\0'){
    //         gps_text += m;
    //     }
    //     sys->wifi_broadcast(gps_text);
    // }
}

/*
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
#if defined(USE_PERIPHERAL_BMP280) || defined(USE_PERIPHERAL_BMP280_LIB)
    sei();
    sys->imu.bmp_update();
    cli();
#endif
}*/