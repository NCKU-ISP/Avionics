#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <EEPROM.h>

typedef struct config {
    uint64_t rtime;
    uint64_t stime;
} config_t;

class Config
{
public:
    Config();
    config_t config;
    int write();
    int read();
};

#endif
