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
#ifdef PARACHUTE_TRIGGER
    pinMode(PIN_TRIGGER_2, OUTPUT);
    digitalWrite(PIN_TRIGGER_2, LOW);
#elif defined(PARACHUTE_SERVO)
    servo.attach(PIN_MOTOR, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
#endif
#ifdef LAUNCH_TRIGGER
    pinMode(PIN_TRIGGER, OUTPUT);
    digitalWrite(PIN_TRIGGER, LOW);
#endif

#ifdef USE_SERVO_CONTROL
    servo[0].attach(PIN_SERVO_1);
    servo[1].attach(PIN_SERVO_2);
    servo[2].attach(PIN_SERVO_3);
    servo[3].attach(PIN_SERVO_4);
#endif
}

SYSTEM_STATE System::init(bool soft_init)
{
    rocket = {.state = ROCKET_READY,
              .fairing = false,
              .ftype = F_SERVO,
              .buzzState = BUZ_LEVEL1};

    rocket.buzzState = buzz(BUZ_LEVEL1);

    setServo(&servo, SERVO_CLOSED_ANGLE);

    // Setup logger
    while (!logger.init()) {
        // buzzer(BUZ_LEVEL0);
    }
#ifdef USE_WIFI_COMMUNICATION
    if (soft_init)
        comms.init();
#endif
    if (soft_init)
        OTA_init();
    // logger.log_code(INFO_LOGGER_INIT, LEVEL_INFO);

    // Setup IMU
    if (imu.init() != ERROR_OK) {
        // logger.log_code(ERROR_IMU_INIT_FAILED, LEVEL_ERROR);
        // buzzer(BUZ_LEVEL0);
        comms.wifi_broadcast(String("ERROR_IMU_INIT_FAILED") + LEVEL_ERROR);
        Serial.println("imu_fail");
        digitalWrite(PIN_BUZZER, 1);
    } else {
        Serial.println("imu initialized success");
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
    Serial.println(WiFi.softAPIP());
}

void System::loop()
{
#ifdef USE_WIFI_COMMUNICATION

    comms.loop();  // Loop for the wifi opertation

    static bool keep = false;
    // Read and react to the command from comms or debugger
    if (comms.message != "") {
        command(comms.message, CMD_WIFI);
    }

    static String serial_cmd = "";
    if (Serial.available() || serial_cmd != "") {
        int c = Serial.read();
        if (c <= 0 || c >= 0xfe) {
        } else if (c != '\n' && !keep) {
            Serial.print((char) c);
            serial_cmd += (char) c;
        } else {
            keep = command(serial_cmd, CMD_SERIAL);
            comms.wifi_broadcast(serial_cmd);
            if (!keep)
                serial_cmd = "";
        }
    }
    if (core_cmd != "") {
        command(core_cmd, CMD_BOTH);
        core_cmd = "";
    }
#ifdef ESP_NOW
    const char *esp_now_msg = fetchESPNOWMessage();
    if (esp_now_msg) {
        command(esp_now_msg, CMD_BOTH);
        clearESPNOWMessage();
    }
#endif
    // servo.write(180);

    imu.bmp_update();

    flight();

    loading_test(&comms.message);

    ArduinoOTA.handle();
#endif

    // ArduinoOTA.handle();

#ifdef USE_GPS_NEO6M
    if (imu.gpsSerial.available()) {
        char m = (char) imu.gpsSerial.read();
        if (m != '\n')
            imu.gpsCode += m;
        else {
            comms.wifi_broadcast(imu.gpsCode);
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

BUZZER_LEVEL System::buzz(BUZZER_LEVEL beep, int times /* = 0 */)
{
#ifdef USE_PERIPHERAL_BUZZER
    const float unit_time = 0.1;  // seconds
    static int counter = 0;
    buzzer.detach();
    switch (beep) {
    case BUZ_LEVEL1:
        buzzer.attach(10 * unit_time, [=]() {
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
            if (times > 0) {
                counter++;
                Serial.println(counter);
                if (counter >= 2 * times) {
                    Serial.println("stop");
                    counter = 0;
                    buzz(rocket.buzzState);
                }
            }
        });
        break;
    case BUZ_LEVEL2:
        buzzer.attach(5 * unit_time, [=]() {
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
            if (times > 0) {
                counter++;
                if (counter >= 2 * times) {
                    counter = 0;
                    buzz(rocket.buzzState);
                }
            }
        });
        break;
    case BUZ_LEVEL3:
        buzzer.attach(2 * unit_time, [=]() {
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
            if (times > 0) {
                counter++;
                if (counter >= 2 * times) {
                    counter = 0;
                    buzz(rocket.buzzState);
                }
            }
        });
        break;
    case BUZ_LEVEL4:
        buzzer.attach(1 * unit_time, [=]() {
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
            if (times > 0) {
                counter++;
                if (counter >= 2 * times) {
                    counter = 0;
                    buzz(rocket.buzzState);
                }
            }
        });
        break;

    default:
        digitalWrite(PIN_BUZZER, LOW);
        break;
    }

    return beep;
#endif
}

void System::trig(int pin, bool trig)
{
#ifdef V3_PIONEER
    digitalWrite(S0, 1);
    digitalWrite(S1, 1);
    digitalWrite(S2, 1);
    digitalWrite(PIN_CD4051, trig);
#else
    digitalWrite(pin, trig);
#endif
}

void System::fairing(int angle)
{
    // ON is true, OFF is false
    bool on_off = angle == openAngle;
#ifdef PARACHUTE_SERVO
    setServo(&servo, angle);
#elif defined(PARACHUTE_TRIGGER)
    trig(PIN_TRIGGER_2, on_off);
#endif
    rocket.fairing = on_off;

    rocket.buzzState = buzz(BUZ_LEVEL4);
}

void System::servoOff()
{
// release the servo to save power
#ifdef PARACHUTE_TRIGGER
    trig(PIN_TRIGGER_2, false);
#elif defined(PARACHUTE_SERVO)
    servo.detach();
#endif
}

void System::setFairingLimit(int close, int open)
{
    closeAngle = close;
    openAngle = open;
}

void System::setServo(Servo *s, int angle)
{
#ifdef PARACHUTE_SERVO
    if (!s->attached())
        s->attach(PIN_MOTOR, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    s->write(angle);
#endif
}

bool System::command(String cmd, CMD_TYPE type)
{
    String msg = "";
    bool keep = false;

    // Fairing command
    if (cmd == "open") {
        fairing(openAngle);
        msg = "open fairing done";
        if (rocket.state == ROCKET_OFFGROUND) {
            logger.log("open", LEVEL_FLIGHT);
            logger.f.close();
            logger.appendFile(logger.file_ext);
        }
    } else if (cmd == "close") {
        fairing(closeAngle);
        msg = "close fairing done";
    } else if (cmd.substring(0, 11) == "set fairing") {
        // cmd:set fairing (open angle) (close angle), ex: set fairing 10 100
        String degree = cmd.substring(12);
        int open = degree.substring(0, degree.indexOf(' ')).toInt();
        int close = degree.substring(degree.indexOf(' ')).toInt();
        if (open > 180 || open < 0 || close > 180 || close < 0) {
            msg = "set fairing failed : angle out of limit, must 0~180";
        } else {
            setFairingLimit(close, open);
            msg = "set fairing done";
        }
    } else if (cmd == "detach") {
        servoOff();
        msg = "servo detached";
    } else if (cmd.substring(0, 5) == "motor") {
#ifdef PARACHUTE_SERVO
        setServo(&servo, cmd.substring(6).toInt());
        msg = "set servo done";
#endif
    }

    else if (cmd.substring(0, 5) == "count") {
        count_down_time = cmd.substring(5).toInt();
        msg = "count-down:" + String(count_down_time);
    }

    // Preflight command
    else if (cmd == "preLaunch" && rocket.state == ROCKET_READY) {
        rocket.state = ROCKET_PREFLIGHT;
        buzzer.attach(0.5, [=]() {
            static int counter = 0;
            counter++;
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));

            if (count_down_time < 3 || counter == 2 * count_down_time) {
                buzzer.detach();
                digitalWrite(PIN_BUZZER, 1);
                buzzer.once(3, [=]() { rocket.buzzState = buzz(BUZ_NONE); });
            }
        });
        msg = "Start count down sequence.";
    }

    // Launch command
    else if (cmd == "launch" && rocket.state == ROCKET_PREFLIGHT) {
        rocket.state = ROCKET_OFFGROUND;
        logger.newFile(LEVEL_FLIGHT);
        comms.wifi_broadcast("launch");
        msg = logger.file_ext + " launch";

        fly_plan.once_ms(release_t, [=]() {
            core_cmd = "open";
            fly_plan.detach();
            fly_plan.once_ms(stop_t, [=]() {
                rocket.buzzState = buzz(BUZ_LEVEL3);
                core_cmd = "stop";
            });
        });
        core_cmd = "stream";
        log.attach_ms(10, [=]() { wait_log = true; });

#ifdef LAUNCH_TRIGGER
        trig(PIN_TRIGGER, true);
#endif
    }

    // Recording command
    else if (cmd == "stop" && rocket.state == ROCKET_OFFGROUND) {
        rocket.state = ROCKET_LANDED;
        logger.f.close();
        msg = "stop," + logger.file_ext + ": recording stopped";

        rocket.buzzState = buzz(BUZ_LEVEL3);
        fly_plan.detach();
        log.detach();
        wait_log = false;
    }

    // Time command
    else if (cmd.substring(0, 5) == "rtime") {
        // Rising time
        release_t = cmd.substring(6).toInt();
        msg = String("Set release time to ") + release_t + "ms";
    } else if (cmd.substring(0, 5) == "stime") {
        // Stop time
        stop_t = cmd.substring(6).toInt();
        msg = String("Set stop time to ") + stop_t + "ms";
    }

    // Restart command
    else if (cmd == "restart") {
        pinMode(16, OUTPUT);
        digitalWrite(16, 0);
    }

    //
    // File manipulation
    //
    else if (cmd == "list") {  // List all file command
        msg = "l," + logger.listFile();
    }

    else if (cmd.substring(0, 4) == "read" &&
             rocket.state != ROCKET_OFFGROUND) {  // Read specific file
        static int pos = 0;
        static bool stream_active;
        if (stream.active()) {
            stream_active = stream.active();
            core_cmd = "nostream";
        } else {
            msg = logger.readFile(cmd.substring(5), &pos);
            if (pos == -1) {
                keep = false;
                pos = 0;
            } else
                keep = true;
            if (!keep && stream_active)
                core_cmd = "stream";
        }
    }

    else if (cmd.substring(0, 6) == "delete") {  // Delete specific file
        msg = cmd.substring(7) + ":" + logger.deleteFile(cmd.substring(7))
                  ? "File deleted"
                  : "Delete failed";
    } else if (cmd == "clear") {  // Delete all the logged data
        msg = logger.clearDataFile();
    } else if (cmd == "info") {  // Info check command
        msg = logger.fsInfo();
    } else if (cmd == "space") {  // Show the remaining space
        msg = logger.remainSpace();
    } else if (cmd == "format") {  // Format the filesystem
        msg = logger.formatFS() ? "Formatted" : "Format failed";
    }

    else if (cmd.substring(0, 4) == "buzz") {
        rocket.buzzState = buzz((BUZZER_LEVEL) cmd.substring(4).toInt());
        msg = cmd;
    }

    else if (cmd == "rocket") {
        msg += "state: " + String(rocket.state) + '\n';
        msg += "fairing: " + (rocket.fairing ? "open" : "closed") + "\n";
        msg += "fairing type: " +
               ((rocket.ftype == F_TRIGGER) ? "trigger" : "servo") + "\n";
        msg += "comms state: " + String(rocket.cState);
        msg += "release at " + String(release_t) + "ms\n";
        msg += "stop at " + String(stop_t) + "ms\n";
    }

    else if (cmd == "connected") {
        buzz(BUZ_LEVEL4, 2);
        comms.message = "";
    } else if (cmd == "disconnected") {
        buzz(BUZ_LEVEL4, 4);
        comms.message = "";
    }

    else if (cmd.substring(0, 5) == "print") {
        msg = cmd.substring(5);
    } else if (cmd == "nostream") {
        stream.detach();
        wait_stream = false;
        msg = "nostream";
    } else if (cmd == "stream") {
        if (!stream.active())
            stream.attach_ms(100, [=]() { wait_stream = true; });
        msg = "stream";
    }

    // soft initialization (initialize without disconnect wifi)
    else if (cmd == "init") {
        init(false);
        msg = "soft init";
    }

    // Print out msg through serial or wifi
    if (msg != "") {
        if (type == CMD_SERIAL || type == CMD_BOTH)
            Serial.println(msg);
        if (type == CMD_WIFI)
            comms.wifi_broadcast(msg, !keep);
        if (type == CMD_BOTH)
            comms.wifi_broadcast(msg, false);
    }

    return keep;
}

void System::flight()
{
    float height = 0, speed = 0;
    String data_head;
#ifdef USE_PERIPHERAL_BMP280
    height = imu.est_altitude;
    speed = imu.velocity;
#endif
    static auto T_start = -1;
    if (rocket.state == ROCKET_OFFGROUND) {
        if (T_start == -1)
            T_start = millis();
        auto T_now = millis();
        auto T_plus = T_now - T_start;
        data_head = String("f,") + T_plus;

        if (imu.pose == ROCKET_FALLING && !rocket.fairing &&
            T_plus > LIFT_OFF_PROTECT_TIME) {
            // fairing(openAngle);
            core_cmd = "open";
        }
    } else {
        data_head = String("s,0");
    }

    if (rocket.state == ROCKET_LANDED) {
        T_start = -1;
    }

    if (wait_log || wait_stream)
        data = data_head + ',' + height + ',' + speed + ',' + imu.pose + ',' +
               comms.dB + ',' + imu.altitude + ',' + imu.seaLevelHpa;
    if (wait_log) {
        logger.log(data, LEVEL_FLIGHT);
        wait_log = false;
    }
    if (wait_stream) {
        command("print" + data, CMD_WIFI);
        wait_stream = false;
    }

    if (speed < -12 && height < 10) {
        core_cmd = "stop";
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
        comms.wifi_broadcast(logger.file_ext + ": recording start");
        start_t = millis();
    }
    if (testing) {
        if (loadcell.is_ready()) {
            String reading = String(loadcell.get_units(1));
            String time = String(millis() - start_t);
            comms.wifi_broadcast(time + "," + reading);
            logger.log(time + "," + reading, LEVEL_FLIGHT);
            Serial.println(reading);
        }
    }
    if (*command == "stop test") {
        digitalWrite(12, 0);
        logger.f.close();
        comms.wifi_broadcast(logger.file_ext + ": recording stop");
        testing = false;
        *command = "";
    }
#endif
}