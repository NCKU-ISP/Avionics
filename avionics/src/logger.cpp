#include "logger.h"
#include "Arduino.h"

#include <SPI.h>
#include <SD.h>
#include "configs.h"

Logger::Logger(){

}

bool Logger::init(){
    if (!SD.begin(LOGGER_SD_CS)){
        // SD card initialization succeed

        // DOTO: Check remaining size
    }

    // Check whether the filename is unique
    String filename = LOGGER_FILENAME;
    String extension = LOGGER_FILE_EXT;
    file_ext = filename + extension;
    for (int i=0; SD.exists(file_ext) ; i++)
        file_ext = filename + String(i) + extension;
    return true;
}

bool Logger::log(){
    // Create file if it is not exist
    sd = SD.open(file_ext, FILE_WRITE);
    if ( sd ){
        sd.close();
    }
}