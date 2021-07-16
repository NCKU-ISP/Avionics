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
#include <configs.h>


// Semaphore to protect the manipulation of SPI bus
SemaphoreHandle_t spiSemaphore = NULL;

System sys;

void setup()
{
    vSemaphoreCreateBinary(spiSemaphore);

    // put your setup code here, to run once:
    sys.init();

    while (sys.init() != SYSTEM_READY) {
        sys.buzzer(BUZ_LEVEL2);
    }
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

    static auto last_log_time = millis();
    if (millis() - last_log_time > LOGGER_LOG_INTERVAL) {
#ifdef USE_PERIPHERAL_BMP280
        sys.imu.bmp_update();
        sys.logger.log(String(sys.imu.altitude), LEVEL_INFO);
#endif
        // sys->logger.lora_send(LORA_ACCEL, sys->imu.mpu_last_update_time,
        //                      &(sys->imu.q));
        // int16_t alt = sys.imu.altitude;
        // sys.logger.lora_info(LORA_BMP, millis() & 0xFFF, alt >> 8, alt &
        // 0xFF);
        // sys.logger.lora_send(LORA_ACCEL, millis() & 0xFFF, sys.imu.altitude);
        last_log_time = millis();
    }

    sys.imu.imu_isr_update();
}