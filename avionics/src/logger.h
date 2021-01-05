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
#include "configs.h"

enum LOG_LEVEL { LEVEL_DEBUG, LEVEL_INFO, LEVEL_WARNING, LEVEL_ERROR };

class Logger
{
private:
    String file_ext;

public:
    Logger();

    /* Open the logger file in SD card and check the ramaining size
     * Return: False if fail to initialize.
     */
    bool init();

    /* Perform logging task */
    bool log(String msg, LOG_LEVEL level = LEVEL_DEBUG);
};

#endif