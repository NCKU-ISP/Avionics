// This program was developed by NCKU ISP team, this program follows
// the BSD 3-Clause License.
// Copyright (c) 2021, Yencheng Chu (sciyen) <e94066157@gs.ncku.edu.tw>
// All rights reserved.

/* ===============================================
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
=============================================== */

#include <Arduino.h>

#include <Core.h>
#include <SX126x-Arduino.h>
#include <configs.h>

#ifdef USE_LORA_COMMUNICATION
#include <Lora.h>
extern LoraCommunication lora;
#endif

// Semaphore to protect the manipulation of SPI bus
SemaphoreHandle_t spiSemaphore = NULL;

TaskHandle_t lora_communication;
TaskHandle_t regular_update;
System sys;
void task_for_lora_communication(void *parameter);

void setup()
{
    vSemaphoreCreateBinary(spiSemaphore);

    // put your setup code here, to run once:
    sys.init();


    while (sys.init() != SYSTEM_READY) {
        sys.buzzer(BUZ_LEVEL2);
        sys.buzzer_update();
    }
#ifdef USE_LORA_COMMUNICATION
    lora.begin();

    xTaskCreatePinnedToCore(
        task_for_lora_communication, /* Function to implement the task */
        "lora_communication",        /* Name of the task */
        10000,                       /* Stack size in words */
        NULL,                        /* Task input parameter */
        0,                           /* Priority of the task */
        &lora_communication,         /* Task handle. */
        0);                          /* Core where the task should run */
#endif
}
int count = 0;
void loop()
{
    static time_t last = millis();
    if ((millis() - last) > 200) {
        last = millis();
        /*#ifdef USE_LORA_COMMUNICATION
                LoraPacketFloat_3d p(count, VectorFloat(0, 0, 0));
                count++;
                if (!lora.send(&p)) {
                    Serial.println("Lora buffer full");
                } else
                    Serial.println("Lora queued");
        #endif*/
        /*
        #ifdef USE_PERIPHERAL_GPS
                gps.update();
                if (gps.number_of_satellites > 0) {
                    Serial.print("satellite: ");
                    Serial.print(gps.number_of_satellites);
                    Serial.print(", longitude: ");
                    Serial.print(gps.longitude);
                    Serial.print(", latitude: ");
                    Serial.println(gps.latitude);
                } else {
                    Serial.println("GPS no signal");
                }
        #endif*/
        LoraPacketSystemState p = sys.pack_system_state();
        lora.send(&p);
    }

    if (sys.imu.pose == ROCKET_RISING) {
        sys.buzzer(BUZ_LEVEL0);
        sys.logger.log("Rising");
    } else if (sys.imu.pose == ROCKET_FALLING) {
        sys.parachute(SERVO_RELEASE_ANGLE);
        sys.buzzer(BUZ_LEVEL1);
        sys.logger.log("Falling");
    }

    static auto last_log_time = millis();
    if (millis() - last_log_time > LOGGER_LOG_INTERVAL) {
        last_log_time = millis();
    }
    /*
    sys.imu.imu_isr_update();
    sys.buzzer_update();*/
    sys.routine();
}

#ifdef USE_LORA_COMMUNICATION
void task_for_lora_communication(void *parameter)
{
    for (;;) {
        lora.update();
    }
}
#endif