#include "logger.h"

Logger::Logger()
#ifdef USE_LORA_COMMUNICATION
    : lora(PIN_LORA_SELECT,    // Port-Pin Output: SPI select
           PIN_LORA_RESET,     // Port-Pin Output: Reset
           PIN_LORA_BUSY,      // Port-Pin Input:  Busy
           PIN_LORA_INTERRUPT  // Port-Pin Input:  Interrupt DIO1
      )
#endif
{
#ifdef USE_LORA_COMMUNICATION
    lora_packet_id = 0;
#endif
}

bool Logger::init()
{
#ifdef USE_SERIAL_DEBUGGER
    Serial.begin(SERIAL_DEBUGGER_BAUDRATE);
#endif

#ifdef USE_PERIPHERAL_SD_CARD
    if (!SD.begin(LOGGER_SD_CS)) {
        // SD card initialization failed
        log_error(ERROR_SD_INIT_FAILED);
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

void Logger::lora_init()
{
#ifdef USE_LORA_COMMUNICATION
    lora.begin(SX126X_PACKET_TYPE_LORA,  // LoRa or FSK, FSK
                                         // currently not supported
               RF_FREQUENCY,             // frequency in Hz
               -3);                      // tx power in dBm

    lora.LoRaConfig(LORA_SPREADING_FACTOR, LORA_BANDWIDTH, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_PAYLOADLENGTH,
                    false,   // crcOn
                    false);  // invertIrq
#endif
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

void Logger::log_error(ERROR_CODE code)
{
    log(String("Error:") + code, LEVEL_ERROR);
}

void Logger::log_info(INFO_CODE code)
{
    log(String("Info:") + code, LEVEL_INFO);
}

#ifdef USE_LORA_COMMUNICATION
void Logger::lora_send(LOG_LORA_MODE mode, int16_t *data)
{
    packet.Packet.id = mode << 12 | lora_packet_id;
    memcpy(packet.Packet.data, data, sizeof(int16_t) * 3);

    lora.Send(packet.raw, 8, SX126x_TXMODE_SYNC);
    lora_packet_id++;
    lora_packet_id &= 0x0FFF;
}
#endif