#include <core.h>

System *sys;

void setup()
{
    delay(3000);
    Serial.begin(115200);
    // Serial.setDebugOutput(true);
    Serial.end();

    sys = new System();
    sys->init();
}

void loop()
{
    sys->loop();
}