#ifndef NEO_GPS_H
#define NEO_GPS_H
#include <Arduino.h>
#include <HardwareSerial.h>

/* This module use NeoGps library which can be found here         *
 * https://github.com/SlashDevin/NeoGPS                           */
#include <NMEAGPS.h>
#include "../../include/configs.h"

// Check that the config files are set up properly

#if !defined(NMEAGPS_PARSE_RMC)
#error You must uncomment NMEAGPS_PARSE_RMC in NMEAGPS_cfg.h!
#endif

#if !defined(GPS_FIX_TIME)
#error You must uncomment GPS_FIX_TIME in GPSfix_cfg.h!
#endif

#if !defined(GPS_FIX_LOCATION)
#error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

#if !defined(GPS_FIX_SPEED)
#error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif

#if !defined(GPS_FIX_SATELLITES)
#error You must uncomment GPS_FIX_SATELLITES in GPSfix_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

class NeoGps
{
private:
    /* Id of the hardware serial port that is going to communicate with gps.*/
    const uint8_t gps_serial_port = 1;

    /* The serial port that is going to communicate with gps module. */
    HardwareSerial GpsSerial;

    NMEAGPS gps;  // This parses the GPS characters

public:
    /* The lat/lon unit in deg/10000000 */
    int32_t latitude;
    int32_t longitude;
    int8_t number_of_satellites;

    struct last_update_t {
        uint8_t hours;
        uint8_t minutes;
        uint8_t seconds;
    } last_update;

    NeoGps();

    void begin(const uint8_t rx_pin = PIN_GPS_RX,
               const uint8_t tx_pin = PIN_GPS_TX,
               const uint16_t baudrate = 9600);

    void update();
};

#endif