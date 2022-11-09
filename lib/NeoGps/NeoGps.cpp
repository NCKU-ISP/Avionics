#include "NeoGps.h"

NeoGps::NeoGps() : GpsSerial(gps_serial_port)
{
    number_of_satellites = -1;
}

void NeoGps::begin(const uint8_t rx_pin,
                   const uint8_t tx_pin,
                   const uint16_t baudrate)
{
    GpsSerial.begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
}

void NeoGps::update()
{
    while (gps.available(GpsSerial)) {
        const gps_fix fix = gps.read();
        if (fix.valid.location) {
            latitude = fix.latitudeL();
            longitude = fix.longitudeL();

            if (fix.valid.satellites)
                number_of_satellites = fix.satellites;

            last_update.hours = fix.dateTime.hours;
            last_update.minutes = fix.dateTime.minutes;
            last_update.seconds = fix.dateTime.seconds;
        } else {
            number_of_satellites = -1;
        }
    }
}