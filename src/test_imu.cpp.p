#include <core.h>

System *sys;
float ax, ay, az, gx, gy, gz, mx, my, mz;

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
    Serial.println("ts	ax	ay	az	wx	wy	wz	mx	my	mz");
}

void loop()
{
    sys->sensor.update();
    ax = sys->sensor.getAcc().x;
    ay = sys->sensor.getAcc().y;
    az = sys->sensor.getAcc().z;
    gx = sys->sensor.getGyro().x;
    gy = sys->sensor.getGyro().y;
    gz = sys->sensor.getGyro().z;
    mx = sys->sensor.getMag().x;
    my = sys->sensor.getMag().y;
    mz = sys->sensor.getMag().z;

    Serial.printf("%ld\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", millis(),ax,ay,az,gx,gy,gz,mx,my,mz);
    // sys->loop();
}