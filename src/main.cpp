#include <core.h>

System *sys;

void setup()
{
    delay(3000);
    Serial.begin(115200);
    // Serial.setDebugOutput(true);
#if (!defined(USE_SERIAL_COMMS)) && (!defined(USE_SERIAL_DEBUGGER))
    Serial.end();
#endif

    sys = new System();
    sys->init();
}

void loop()
{
    sys->loop();
}