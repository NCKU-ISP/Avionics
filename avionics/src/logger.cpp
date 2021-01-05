#include "logger.h"

Logger::Logger() {}

bool Logger::init()
{
#ifdef USE_SERIAL_DEBUGGER
    Serial.begin(SERIAL_DEBUGGER_BAUDRATE);
#endif

#ifdef USE_PERIPHERAL_SD_CARD
    if (!SD.begin(LOGGER_SD_CS)) {
// SD card initialization failed
#ifdef USE_SERIAL_DEBUGGER
        Serial.println("Failed to initialize SD card!");
#endif
        // DOTO: Check remaining size
        return false;
    }

    // Check whether the filename is unique
    String filename = LOGGER_FILENAME;
    String extension = LOGGER_FILE_EXT;
    file_ext = filename + extension;
    for (int i = 0; SD.exists(file_ext); i++)
        file_ext = filename + String(i) + extension;
#endif
    return true;
}

bool Logger::log(String msg, LOG_LEVEL level)
{
    // Adding prefix message
    String prefix = "";
    switch (level) {
    case LEVEL_DEBUG:
        prefix = "[DEBUG]  :";
        break;
    case LEVEL_INFO:
        prefix = "[INFO]   :";
        break;
    case LEVEL_WARNING:
        prefix = "[WARNING]:";
        break;
    case LEVEL_ERROR:
        prefix = "[ERROR]  :";
        break;
    }
    prefix += String(millis()) + ':';

#ifdef USE_PERIPHERAL_SD_CARD
    // Create file if it is not exist
    File sd;
    sd = SD.open(file_ext, FILE_WRITE);
    if (sd) {
        sd.println(prefix + msg);
        sd.close();
    }
#endif

#ifdef USE_SERIAL_DEBUGGER
    Serial.println(prefix + msg);
#endif
}