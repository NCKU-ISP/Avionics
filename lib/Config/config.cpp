#include "config.h"

Config::Config()
{
    EEPROM.begin(sizeof(config_t));
    this->read();
}

int Config::read()
{
    uint8_t *ptr = (uint8_t *) (&this->config);
    for (size_t i = 0; i < sizeof(config_t); i++)
        *(ptr + i) = EEPROM.read(i);
    return 0;
}

int Config::write()
{
    uint8_t *ptr = (uint8_t *) (&this->config);
    for (size_t i = 0; i < sizeof(config_t); i++)
        EEPROM.write(i, *(ptr + i));
    return EEPROM.commit();
}