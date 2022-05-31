/*
 * This library deals with the logging
 * Including
 * 1. Log into SD card
 * 2. Log with communication module
 *
 * Require:
 * Return:
 */

#ifndef _LOGGER_H
#define _LOGGER_H

#include <SD.h>
#include <SPI.h>
#include "Arduino.h"
#include "configs.h"z

#ifdef USE_LORA_COMMUNICATION
#include <SX126x.h>
#endif

enum LOG_LEVEL { LEVEL_DEBUG, LEVEL_INFO, LEVEL_WARNING, LEVEL_ERROR };
enum LOG_LORA_MODE { LORA_ACCEL, LORA_GYRO, LORA_COMMAND };

typedef union {
    struct {
        uint16_t id;
        int16_t data[3];
    } Packet;
    uint8_t raw[8];
} LoraPacket;

class Logger
{
private:
    String file_ext;
#ifdef USE_LORA_COMMUNICATION
    LoraPacket packet;
    uint16_t lora_packet_id;
#endif

public:
#ifdef USE_LORA_COMMUNICATION
    SX126x lora;
#endif

    Logger();

    /* Open the logger file in SD card and check the ramaining size
     * Return: False if fail to initialize.
     */
    bool init();

    void lora_init();

    /* Perform logging task */
    void log(String msg, LOG_LEVEL level = LEVEL_DEBUG);

    /* Log existing error code or info code */
    void log_code(int code, LOG_LEVEL level);

#ifdef USE_LORA_COMMUNICATION
    void lora_send(LOG_LORA_MODE mode, int16_t *data);
#endif
};

#endif