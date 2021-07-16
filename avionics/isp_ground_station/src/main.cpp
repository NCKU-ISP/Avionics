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

#include <Lora.h>
#include <SX126x-Arduino.h>
#include <configs.h>

extern LoraCommunication lora;
TaskHandle_t lora_communication;
void task_for_lora_communication(void *parameter);

void setup()
{
    Serial.begin(115200);
    lora.begin();
    xTaskCreatePinnedToCore(
        task_for_lora_communication, /* Function to implement the task */
        "lora_communication",        /* Name of the task */
        10000,                       /* Stack size in words */
        NULL,                        /* Task input parameter */
        0,                           /* Priority of the task */
        &lora_communication,         /* Task handle. */
        0);
    Serial.println("Start");
}

void loop()
{
    uint8_t buf[LORA_PACKET_SIZE];
    if (lora.q_rx.pop(buf)) {
        // receive new data
        const uint8_t size = buf[0];
        switch (size) {
        case LoraPacketFloat_3d::size + 3: {
            LoraPacketFloat_3d p(buf);
            Serial.print("[Vector3d]");
            Serial.print(p.id);
            Serial.print(": ");
            Serial.print(p.timestamp * 16);
            Serial.print(": ");
            Serial.print(p.data.vector[0]);
            Serial.print(", ");
            Serial.print(p.data.vector[1]);
            Serial.print(", ");
            Serial.println(p.data.vector[2]);
            break;
        }
        case LoraPacketMessage::size + 3: {
            LoraPacketMessage p(buf);
            Serial.print("[Message ]");
            Serial.print(p.id);
            Serial.print(": ");
            Serial.print(p.timestamp * 16);
            Serial.print(": ");
            Serial.println(p.msg);
            break;
        }
        case LoraPacketSystemState::size + 3: {
            // system state
            LoraPacketSystemState p(buf);
            const float battery = ((float) p.data.state.battery / 256.0) *
                                      (BATTERY_VOLATGE_UPPER_BOUND -
                                       BATTERY_VOLATGE_LOWER_BOUND) +
                                  BATTERY_VOLATGE_LOWER_BOUND;
            Serial.print("[  State ]");
            Serial.print(p.id);
            Serial.print(": ");
            Serial.print(p.timestamp * 16);
            Serial.print(": Battery:");
            Serial.print(battery);
            Serial.print(", Satellite:");
            Serial.print(p.data.state.satellite);
            Serial.print(", Altitude:");
            Serial.print(p.data.state.height);
            Serial.print(", Speed:");
            Serial.print(p.data.state.speed);
            Serial.print(", AngularVel:");
            Serial.print(p.data.state.angular_velocity);
            Serial.print(", Longitude:");
            Serial.print(p.data.state.longitude);
            Serial.print(", Latitude:");
            Serial.println(p.data.state.latitude);
            break;
        }
        default:
            Serial.print("Unknown data type, size; ");
            Serial.println(size);
            for (int i = 0; i < LORA_PACKET_SIZE; i++) {
                Serial.print(buf[i]);
                Serial.print(',');
            }
            Serial.println(' ');
        }
    }
}

void task_for_lora_communication(void *parameter)
{
    for (;;) {
        lora.update();
    }
}