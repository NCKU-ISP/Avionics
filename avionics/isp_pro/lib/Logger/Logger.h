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

#include <../../include/configs.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include <../Helper_3dmath/Helper_3dmath.h>

#ifdef USE_LORA_COMMUNICATION
#include "Lora.h"
#ifdef USE_LORA_COMMUNICATION
#endif
#endif

enum LOG_LEVEL { LEVEL_DEBUG, LEVEL_INFO, LEVEL_WARNING, LEVEL_ERROR };
enum LOG_LORA_MODE { LORA_ACCEL, LORA_BMP, LORA_GYRO, LORA_COMMAND };

class Logger
{
private:
    bool sd_inited;
    String file_ext;

public:
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

    void log_test(int code, LOG_LEVEL level);
};

#endif