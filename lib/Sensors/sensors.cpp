#include "sensors.h"

SENSOR::SENSOR() {}

bool SENSOR::init()
{
    return (init_bmp() && init_imu());
}

bool SENSOR::init_imu()
{
    if (imu.begin() != INV_SUCCESS) {
        Serial.println("IMU initialize failed");
        return 0;
    }
    imu.setSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);

    // Set accelerometer and gyroscope full scale range
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(2000);  // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(16);  // Set accel to 16 g
    // MPU-9250's magnetometer FSR is set at +/- 4912 uT

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(100);  // Set sample rate to 10Hz

    // imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
    //           DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
    //           100);

    Serial.println("IMU initialize successfully");
    return 1;
}

bool SENSOR::init_bmp()
{
    // bmp.reset();
    if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
        Serial.println("BMP initialize failed");
        return 0;
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,   /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X2,   /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,    /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. (ms) */

    rate_bmp = 1000;
    Serial.println("BMP initialize successfully");
    calibrate_bmp();

    altitudeKalmanFilter = new SimpleKalmanFilter(1, 1, 0.01);
    return 1;
}

void SENSOR::calibrate_bmp()
{
    float p = 0, t = 0;
    for (int i = 0; i < IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING; i++) {
        p += bmp.readPressure();
        t += bmp.readTemperature();
        delay(20);
    }
    pressure_Pa = p / IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING;
    pressure_HPa = pressure_Pa / 100;

    Serial.println("BMP calibration complete");
}

bool SENSOR::init_gps()
{
    gps.x = 0;
    gps.y = 0;
    gps.z = 0;
    return 1;
}

void SENSOR::update()
{
    update_imu();
    update_bmp();
    // update_gps();
}

fvec_t SENSOR::getAcc()
{
    return acc;
}
fvec_t SENSOR::getGyro()
{
    return gyro;
}
fvec_t SENSOR::getMag()
{
    return mag;
}
fvec_t SENSOR::getGps()
{
    return gps;
}

float SENSOR::getBmpAltitude()
{
    return altitude_bmp;
}

float SENSOR::getBmpVelocity()
{
    return velocity_bmp;
}

float SENSOR::getPressure(uint8_t p_type)
{
    return p_type == HPa ? pressure_HPa : pressure_Pa;
}

void SENSOR::update_imu()
{
    // Update the imu data
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    acc.x = imu.calcAccel(imu.ax);
    acc.y = imu.calcAccel(imu.ay);
    acc.z = imu.calcAccel(imu.az);
    gyro.x = imu.calcGyro(imu.gx);
    gyro.y = imu.calcGyro(imu.gy);
    gyro.z = imu.calcGyro(imu.gz);
    mag.x = imu.calcMag(imu.mx);
    mag.y = imu.calcMag(imu.my);
    mag.z = imu.calcMag(imu.mz);
}

bool SENSOR::update_bmp()
{
    static float altitude_last = 0, est_altitude_last = 0;
    static unsigned long T = millis();
    altitude_bmp = bmp.readAltitude(pressure_HPa);
    if (altitude_bmp != altitude_last) {
        auto T_now = millis();
        velocity_bmp = (altitude_bmp - altitude_last) / (1 / rate_bmp);
        altitude_last = altitude_bmp;

        altitude_estimate = altitudeKalmanFilter->updateEstimate(altitude_bmp);
        velocity_estimate = (altitude_estimate - est_altitude_last) /
                            ((float) (T_now - T) / 1000);
        est_altitude_last = altitude_estimate;

        T = T_now;
        return 1;
    }

    if (velocity_estimate > IMU_RISING_CRITERIA) {
        // Rocket rising
        pose = ROCKET_RISING;
    } else if (velocity_estimate < IMU_FALLING_CRITERIA) {
        // Rocket falling
        pose = ROCKET_FALLING;
    } else {
        pose = ROCKET_UNKNOWN;
    }
    return 0;
}

// x    : input
// y    : last output
// fc   : cut-off frequency
// fs   : sampling frequency
float SENSOR::LPF(float x, float y, float fc, float fs)
{
    float tau = 1 / (2.0 * PI * fc);
    float alpha = (1.0 / fs) / (tau + (1.0 / fs));

    return y + alpha * (x - y);
}