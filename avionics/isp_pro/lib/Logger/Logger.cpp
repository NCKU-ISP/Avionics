#include "Logger.h"

// Semaphore to protect the manipulation of SPI bus
extern SemaphoreHandle_t spiSemaphore;

Logger::Logger()
{
    sd_inited = false;
}

bool Logger::init()
{
#ifdef USE_SERIAL_DEBUGGER
    Serial.begin(SERIAL_DEBUGGER_BAUDRATE);
#endif

#ifdef USE_PERIPHERAL_SD_CARD
    if (!SD.begin(LOGGER_SD_CS)) {
        // SD card initialization failed
        // log_error(ERROR_SD_INIT_FAILED);
        log_code(ERROR_SD_INIT_FAILED, LEVEL_ERROR);
        // DOTO: Check remaining size
        return false;
    }
    sd_inited = true;
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return false;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    // Check whether the filename is unique
    Serial.println("Checking for sd file\n");
    String filename = LOGGER_FILENAME;
    String extension = LOGGER_FILE_EXT;
    file_ext = filename + extension;
    for (int i = 0; SD.exists(file_ext); i++)
        file_ext = filename + String(i) + extension;
    Serial.println(file_ext);
#endif
    return true;
}

void Logger::log(String msg, LOG_LEVEL level)
{
    // Adding prefix message
    String prefix = "";
    switch (level) {
    case LEVEL_DEBUG:
        prefix = "D:";
        break;
    case LEVEL_INFO:
        prefix = "I:";
        break;
    case LEVEL_WARNING:
        prefix = "W:";
        break;
    case LEVEL_ERROR:
        prefix = "E:";
        break;
    }
    prefix += String(millis()) + ',';

#ifdef USE_PERIPHERAL_SD_CARD
    // Create file if it is not exist
    if (sd_inited) {
        xSemaphoreTake(spiSemaphore, portMAX_DELAY);
        File sd;
        sd = SD.open(file_ext, FILE_APPEND);
        if (sd) {
            sd.println(prefix + msg);
            sd.close();
        } else {
            Serial.println("SD writing failed");
        }
        xSemaphoreGive(spiSemaphore);
    }
#endif

#ifdef USE_SERIAL_DEBUGGER
    Serial.println(prefix + msg);
#endif
}

void Logger::log_code(int code, LOG_LEVEL level)
{
    log(String(code), level);
}

void Logger::log_test(int code, LOG_LEVEL level) {}