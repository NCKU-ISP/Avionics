#include "sensors.h"

#ifdef USE_PERIPHERAL_MPU6050
MPU9250 IMU::mpu(Wire, IMU_MPU_ADDR);

Vector3f IMU::acc = {0, 0, 0};   // Acceleration
Vector3f IMU::gyro = {0, 0, 0};  // Gyroscope
Vector3f IMU::mag = {0, 0, 0};   // Magnetometer
#endif

/*
double IMU::temper = 0;   // Temperature
double IMU::pressure = 0; // Pressure
double IMU::altitude = 0; // Altitude
double IMU::sea_level_pressure = 0;
*/
IMU::IMU()
{
    state = IMU_ERROR;

#ifdef USE_PERIPHERAL_MPU6050
    // Failed to initialize MPU9250
    if (mpu.begin() < 0)
        return;

    /* MPU 9250 settings */
    // setting the accelerometer full scale range to +/-8G
    mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);

    // setting the gyroscope full scale range to +/-500 deg/s
    mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);

    // setting Digital Low Pass Filter (DLPF) bandwidth to 184 Hz
    mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);

    // setting SRD to 0 for a 1 kHz update rate
    // magnetometer is fixed to 100 Hz if SRD <= 9
    mpu.setSrd(0);

    mpu.enableDataReadyInterrupt();

    // Interrupt setting for mpu9250
    pinMode(PIN_IMU_INT, INPUT);
    attachInterrupt(PIN_IMU_INT, imu_isr_update, RISING);
#endif

#ifdef USE_PERIPHERAL_BMP280
    /* BMP 280 settings */
    if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))
        return;

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. (ms) */
#endif

    state = IMU_OK;
}

IMU_STATE IMU::init()
{
    state = IMU_ERROR;

#ifdef USE_MPU_ISP_INTERFACE
    // Chip select pin for MPU9250
    pinMode(PIN_SPI_CS_IMU, OUTPUT);
    digitalWrite(PIN_SPI_CS_IMU, HIGH);
#endif

#ifdef USE_PERIPHERAL_BMP280
    // Get sea level information
    double p = 0, t = 0;
    for (int i = 0; i < IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING; i++) {
        p += bmp.readPressure();
        t += bmp.readTemperature();
        delay(20);
    }
    // Assuming the initializing altitude is sea level
    seaLevelHpa = p / IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING;
// temper = t / IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING;
#endif

    state = IMU_OK;
    return state;
}

#ifdef USE_PERIPHERAL_MPU6050
void IMU::acceleration_filter() {}

void IMU::gyro_filter() {}

void IMU::imu_isr_update()
{
    mpu.readSensor();
    acc.x = mpu.getAccelX_mss();
    acc.y = mpu.getAccelY_mss();
    acc.z = mpu.getAccelZ_mss();

    gyro.x = mpu.getGyroX_rads();
    gyro.y = mpu.getGyroY_rads();
    gyro.z = mpu.getGyroZ_rads();

    mag.x = mpu.getMagX_uT();
    mag.y = mpu.getMagY_uT();
    mag.z = mpu.getMagZ_uT();
}
#endif

float IMU::altitude_filter(float v)
{
    static float decay = v;
    decay = IMU_ALTITUDE_SMOOTHING_CONSTANT * decay +
            (1 - IMU_ALTITUDE_SMOOTHING_CONSTANT) * v;
    return decay;
}

/* The criteria to determine launching state:
 * 1. The average first derivative of altitude is larger than
 */
void IMU::bmp_update()
{
    altitude = altitude_filter(bmp.readAltitude(seaLevelHpa));
    static float lastAltitude = altitude;

    // times 1000 because unit changes from ms to s
    float derivative =
        1000 * (altitude - lastAltitude) / IMU_BMP_SAMPLING_PERIOD;

    // Updating altitude record
    lastAltitude = altitude;

    if (derivative > IMU_RISING_CRITERIA) {
        // Rocket rising
    } else if (derivative < IMU_FALLING_CRITERIA) {
        // Rocket falling
    }
}