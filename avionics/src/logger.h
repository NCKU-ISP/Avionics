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

#include "Arduino.h"
#include <SPI.h>
#include <SD.h>

class Logger{
private:
    File sd;
    String file_ext;
public:
    Logger();

    /* Open the logger file in SD card and check the ramaining size 
     * Return: False if fail to initialize.
     */
    bool init();

    /* Perform logging task */
    bool log();
};

#endif