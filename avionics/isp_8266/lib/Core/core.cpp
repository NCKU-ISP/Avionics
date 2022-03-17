#include "core.h"
#include <Arduino.h>

#ifdef USE_DUAL_SYSTEM_WATCHDOG
SYSTEM_STATE System::state = SYSTEM_UP;
#endif

System::System()
    : logger(),
      imu()
#ifdef USE_WIFI_COMMUNICATION
      ,
      comms()  // Initialize wifi communication object
#endif
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
#ifdef V3_PIONEER
#else
    pinMode(PIN_TRIGGER, OUTPUT);
    trig(LOW);
    pinMode(PIN_MOTOR, OUTPUT);
#endif
}

SYSTEM_STATE System::init()
{
    // Setup logger
    while (!logger.init()) {
        // buzzer(BUZ_LEVEL0);
    }
#ifdef USE_WIFI_COMMUNICATION
    comms.init();
#endif

    // OTA_init();
    // logger.log_code(INFO_LOGGER_INIT, LEVEL_INFO);

    // Setup IMU
    while (imu.init() != ERROR_OK) {
        // logger.log_code(ERROR_IMU_INIT_FAILED, LEVEL_ERROR);
        buzzer(BUZ_LEVEL0);
        wifi_broadcast(String("ERROR_IMU_INIT_FAILED") + LEVEL_ERROR);
    }
// // logger.log_info(INFO_IMU_INIT);
// // logger.log_code(INFO_IMU_INIT, LEVEL_INFO);

// // Servo position inialization
// parachute(SERVO_INITIAL_ANGLE);
// logger.log_code(INFO_SERVO_INIT, LEVEL_INFO);

// // Lora initialization
// logger.lora_init();
// logger.log_code(INFO_LORA_INIT, LEVEL_INFO);

// // buzzer(BUZ_LEVEL3);

// // Setup core update
// logger.log_code(INFO_ALL_SYSTEM_INIT, LEVEL_INFO);

#ifdef ENGINE_LOADING_TEST
    pinMode(12, OUTPUT);
    digitalWrite(12, 0);

    loadcell.begin(DT_PIN, SCK_PIN);

    Serial.println("Before setting up the loadcell:");

    Serial.println(loadcell.get_units(5),
                   0);  // Data before setting the scale factor

    loadcell.set_scale(SCALE_FACTOR);  // Set the scale factor
    loadcell.tare();                   // Set origin

    Serial.println("After setting up the loadcell:");

    Serial.println(loadcell.get_units(5),
                   0);  // Data after setting the scale factor

    Serial.println("Ready!");
#endif

    return SYSTEM_READY;
}

void System::OTA_init()
{
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_FS
            type = "filesystem";
        }

        // NOTE: if updating FS this would be the place to unmount FS using
        // FS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });
    ArduinoOTA.begin();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}


bool System::wifi_send(uint8_t num, String payload)
{
#ifdef USE_WIFI_COMMUNICATION
    return comms.webSocket.sendTXT(num, payload);
#endif
}
bool System::wifi_send(uint8_t num, const char *payload)
{
#ifdef USE_WIFI_COMMUNICATION
    return comms.webSocket.sendTXT(num, payload);
#endif
}

bool System::wifi_broadcast(String payload)
{
#ifdef USE_WIFI_COMMUNICATION
    return comms.webSocket.broadcastTXT(payload);
#endif
}
bool System::wifi_broadcast(const char *payload)
{
#ifdef USE_WIFI_COMMUNICATION
    return comms.webSocket.broadcastTXT(payload);
#endif
}


void System::loop()
{
#ifdef USE_WIFI_COMMUNICATION

    comms.loop();  // Loop for the wifi opertation

    command(&comms.message);

    imu.bmp_update();

    flight();

    loading_test(&comms.message);
#endif

// ArduinoOTA.handle();

#ifdef USE_GPS_NEO6M
    if (imu.gpsSerial.available()) {
        char m = (char) imu.gpsSerial.read();
        if (m != '\n')
            imu.gpsCode += m;
        else {
            wifi_broadcast(imu.gpsCode);
            imu.gpsCode = "";
        }
    }
#endif
}

#ifdef USE_DUAL_SYSTEM_WATCHDOG
WATCHDOG_STATE System::check_partner_state()
{
    if (millis() - command_update_time > WATCH_SPI_TIMEOUT)
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
#ifdef V3_PIONEER
    digitalWrite(S0, 1);
    digitalWrite(S1, 1);
    digitalWrite(S2, 1);
    digitalWrite(PIN_CD4051, trig);
#else
    digitalWrite(PIN_TRIGGER, trig);
#endif
}

void System::fairingOpen(int angle)
{
    // trig(true);
    servo.attach(PIN_MOTOR);
    servo.write(angle);
    Serial.println("open fairing done.");
    wifi_broadcast("open fairing done.");
    if (start)
        logger.log("open", LEVEL_FLIGHT);
}

// Barely used (Just in case)
void System::fairingClose(int angle)
{
    // trig(false);
    servo.attach(PIN_MOTOR);
    servo.write(angle);
    Serial.println("close fairing done.");
    wifi_broadcast("close fairing done.");
}

void System::fairingServoOff()
{
    // release the servo to save power
    servo.detach();
    trig(false);
}

void System::setFairingLimit(int close, int open)
{
    closeAngle = close;
    openAngle = open;
}

void System::setMotor(int motor, int angle)
{
    servo.detach();
    if (motor != 0) {
        // digitalWrite(S0, 0);
        // digitalWrite(S1, 0);
        // digitalWrite(S2, 1);
        // servo.attach(13);
    } else
        servo.attach(14);
    servo.write(angle);
    String message = "Set" + motor + String("motor to ") + angle + String(" °");
    Serial.println(message);
    wifi_broadcast(message);
    // digitalWrite(S0, 0);
    // digitalWrite(S1, 0);
    // digitalWrite(S2, 0);
    //
}

void System::command(String *command)
{
    // Fairing command
    if ((*command).equals("open")) {
        fairingOpen(openAngle);
        (*command) = "";
    } else if ((*command).equals("close")) {
        fairingClose(closeAngle);
        (*command) = "";
    } else if ((*command).indexOf("set fairing") != -1) {
        int open = (*command).substring((*command).indexOf(":")).toInt();
        (*command).remove((*command).indexOf(":"));
        int close = (*command).toInt();
        if (open > 180 || open < 0 || close > 180 || close < 0) {
            Serial.println(
                "set fairing failed : angle out of limit, must 0~180");
            wifi_broadcast(
                "set fairing failed : angle out of limit, must 0~180");
        } else {
            setFairingLimit(close, open);
            Serial.println("set fairing done.");
            wifi_broadcast("set fairing done.");
        }
        (*command) = "";
    }

    // Launch command
    if (comms.message == "launch") {
        start = true;
        logger.newFile(LEVEL_FLIGHT);
        wifi_broadcast(logger.file_ext);
        wifi_broadcast("launch");
        comms.message = "";
    }

    // Recording command
    if (comms.message == "stop" && start) {
        start = false;
        logger.f.close();
        wifi_broadcast(logger.file_ext + ": recording stopped");
        comms.message = "";
    }

    // Time command
    if (comms.message.substring(0, 5) == "rtime") {
        comms.message.remove(0, 6);
        release_t = comms.message.toInt();
        wifi_broadcast(String("Set release time to ") + release_t + "ms");
        comms.message = "";
    } else if (comms.message.substring(0, 5) == "stime") {
        comms.message.remove(0, 6);
        stop_t = comms.message.toInt();
        wifi_broadcast(String("Set stop time to ") + stop_t + "ms");
        comms.message = "";
    }

    // Restart command
    if (comms.message == "restart") {
        pinMode(16, OUTPUT);
        digitalWrite(16, 0);
    }

    // Clear data file command
    if (comms.message == "clear") {
        wifi_broadcast(logger.clearDataFile());
        comms.message = "";
    }

    // Info check command
    if (comms.message == "info") {
        wifi_broadcast(logger.fsInfo());
        comms.message = "";
    } else if (comms.message == "space") {
        wifi_broadcast(logger.remain_space());
        comms.message = "";
    }

    if (comms.message.indexOf("read") != -1) {
        static int position = 0;
        if (position != -1)
            wifi_broadcast(
                logger.readFile(comms.message.substring(5), &position).c_str());
        else {
            position = 0;
            comms.message = "";
        }
    }
}

void System::flight()
{
    float height = 0, speed = 0;
    String series = "";
#ifdef USE_PERIPHERAL_BMP280
    height = imu.est_altitude;
    speed = imu.velocity;
#endif
    // Serial.printf("%.2f, %.2f\n", height, speed);
    // series = String("s");
    auto T_now = millis();
    static auto T_send = T_now;
    static bool first = true;
    static bool opened = false;
    static bool beeping = false;
    if (!first && !start)
        first = true;

    if (start) {
        static auto T_100ms = T_now, T_10ms = T_now, T_release = T_now,
                    T_stop = T_now, T_start = T_now, T_beep = T_now;
        if (first) {
            T_100ms = T_now;
            T_10ms = T_now;
            T_release = T_now;
            T_stop = T_now;
            T_start = T_now;
            opened = false;
            beeping = true;
        }
        first = false;

        series = String("f,") + (T_now - T_start) + ',' + height + ',' + speed +
                 ',' + imu.pose + ',' + comms.dB;
        if (T_now - T_10ms >= 10) {
            // Serial.println((T_now - T_start)/1000);
            logger.log(series, LEVEL_FLIGHT);
            T_10ms = T_now;
        }
        if (T_now - T_100ms >= 100) {
            // Serial.println(series);
            wifi_broadcast(series);
            T_100ms = T_now;
        }
        if (T_now - T_release >= (unsigned) release_t) {
            fairingOpen(SERVO_RELEASE_ANGLE);
            T_release = T_now;
            opened = true;
        }
        if (T_now - T_stop >= (unsigned) stop_t) {
            start = false;
            wifi_broadcast(logger.file_ext + ": recording stopped");
        }
        if (imu.pose == ROCKET_FALLING && !opened) {
            Serial.println(".....................");
            opened = true;
            fairingOpen(SERVO_RELEASE_ANGLE);
        }
        if (imu.pose == ROCKET_FALLING && T_now - T_beep > 1000) {
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
            T_beep = T_now;
        } else if (imu.pose == ROCKET_UNKNOWN && T_now - T_beep > 100 && opened) {
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
            T_beep = T_now;
        }
    } else if (T_now - T_send > 100) {
        series = String("s, 0") + ',' + height + ',' + speed + ',' + imu.pose +
                 ',' + comms.dB;
        wifi_broadcast(series);
        T_send = T_now;
    }
    static auto T_beeping = T_now;
    if (T_now - T_beeping > 500 && beeping && !start) {
        digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
        T_beeping = T_now;
    }
}

void System::loading_test(String *command)
{
#ifdef ENGINE_LOADING_TEST
    static bool testing = false;
    static auto start_t = millis();
    if (*command == "test") {
        digitalWrite(12, 1);
        testing = true;
        *command = "";
        logger.newFile(LEVEL_FLIGHT);
        wifi_broadcast(logger.file_ext + ": recording start");
        start_t = millis();
    }
    if (testing) {
        if (loadcell.is_ready()) {
            String reading = String(loadcell.get_units(1));
            String time = String(millis() - start_t);
            wifi_broadcast(time + "," + reading);
            logger.log(time + "," + reading, LEVEL_FLIGHT);
            Serial.println(reading);
        }
    }
    if (*command == "stop test") {
        digitalWrite(12, 0);
        logger.f.close();
        wifi_broadcast(logger.file_ext + ": recording stop");
        testing = false;
        *command = "";
    }
#endif
}