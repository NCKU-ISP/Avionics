#include "sensors.h"

SENSOR::SENSOR() {}

bool SENSOR::init()
{
    uint8_t success = 0;
    success |= init_bmp();
    success != init_imu();
    return success;
}

bool SENSOR::init_imu()
{
#ifdef USE_GY91_MPU9250
    if (imu.begin() != INV_SUCCESS) {
        Serial.println("IMU initialize failed");
        return 1;
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
    imu.setSampleRate(1000);  // Set sample rate to 1kHz

    imu.setCompassSampleRate(100);

    // imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
    //           DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
    //           100);
    calibrate_imu();
    Serial.println("IMU initialize successfully");
#endif
    return 0;
}

void SENSOR::calibrate_imu()
{
    acc_bias.x = 0.481620016135836;
    acc_bias.y = 0.342010466830107;
    acc_bias.z = -0.140797227553077;
    acc_scale.x = 0.999321184282955;
    acc_scale.y = 1.00502079325252;
    acc_scale.z = 0.995680172528931;

    gyro_bias.x = -7.63314044250000;
    gyro_bias.y = -1.11030489300000;
    gyro_bias.z = -3.81207332550001;
    gyro_scale.x = 1;
    gyro_scale.y = 1;
    gyro_scale.z = 1;

    mag_bias.x = 54.4470230785505;
    mag_bias.y = 55.7092688360328;
    mag_bias.z = -23.9733015260566;
    mag_scale.x = 1;
    mag_scale.y = 1;
    mag_scale.z = 1;
}

bool SENSOR::init_bmp()
{
#ifdef USE_PERIPHERAL_BMP280
    // bmp.reset();
    if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
        Serial.println("BMP initialize failed");
        return 1;
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
#endif
    return 0;
}

void SENSOR::calibrate_bmp()
{
#ifdef USE_PERIPHERAL_BMP280
    float p = 0, t = 0;
    for (int i = 0; i < IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING; i++) {
        p += bmp.readPressure();
        t += bmp.readTemperature();
        delay(20);
    }
    pressure_Pa = p / IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING;
    pressure_HPa = pressure_Pa / 100;

    Serial.println("BMP calibration complete");
#endif
}

bool SENSOR::init_gps()
{
    gps.x = 0;
    gps.y = 0;
    gps.z = 0;
    return 0;
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
#ifdef USE_PERIPHERAL_BMP280
    return p_type == HPa ? pressure_HPa : pressure_Pa;
#endif
    return 0;
}

void SENSOR::update_imu()
{
#ifdef USE_GY91_MPU9250
    // Update the imu data
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    acc.x = acc_scale.x * imu.calcAccel(imu.ax) - acc_bias.x;
    acc.y = acc_scale.y * imu.calcAccel(imu.ay) - acc_bias.y;
    acc.z = acc_scale.z * imu.calcAccel(imu.az) - acc_bias.z;
    gyro.x = gyro_scale.x * imu.calcGyro(imu.gx) - gyro_bias.x;
    gyro.y = gyro_scale.y * imu.calcGyro(imu.gy) - gyro_bias.y;
    gyro.z = gyro_scale.z * imu.calcGyro(imu.gz) - gyro_bias.z;
    mag.x = mag_scale.x * imu.calcMag(imu.mx) - mag_bias.x;
    mag.y = mag_scale.y * imu.calcMag(imu.my) - mag_bias.y;
    mag.z = mag_scale.z * imu.calcMag(imu.mz) - mag_bias.z;
#endif
}

bool SENSOR::update_bmp()
{
#ifdef USE_PERIPHERAL_BMP280
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
        return 0;
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
#endif
    return 1;
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