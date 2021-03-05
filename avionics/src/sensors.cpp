#include "sensors.h"

/*#ifdef USE_PERIPHERAL_MPU6050
MPU9250 IMU::mpu(Wire, IMU_MPU_ADDR);

Vector3f IMU::acc = {0, 0, 0};   // Acceleration
Vector3f IMU::gyro = {0, 0, 0};  // Gyroscope
Vector3f IMU::mag = {0, 0, 0};   // Magnetometer
#endif/*

/*
double IMU::temper = 0;   // Temperature
double IMU::pressure = 0; // Pressure
double IMU::altitude = 0; // Altitude
double IMU::sea_level_pressure = 0;
*/
IMU::IMU()
{
    state = IMU_ERROR;
    pose = ROCKET_UNKNOWN;

    state = IMU_OK;
}

IMU_STATE IMU::init()
{
    state = IMU_ERROR;

#ifdef USE_PERIPHERAL_MPU6050
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having
                            // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    mpu.initialize();
    pinMode(PIN_IMU_INT, INPUT);

    if (mpu.testConnection())
        Serial.println("MPU initialization error");

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // mpu.setRate();
    // mpu.setDLPFMode();
    // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    // mpu.setDHPFMode();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(57);
    mpu.setYGyroOffset(2);
    mpu.setZGyroOffset(1);
    mpu.setXAccelOffset(-2424);  // 1688 factory default for my test chip
    mpu.setYAccelOffset(879);    // 1688 factory default for my test chip
    mpu.setZAccelOffset(1064);   // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), dmpDataReady,
        //                RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to
        // use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
#endif

#ifdef USE_PERIPHERAL_BMP280
    /* BMP 280 settings */
    if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))
        return state;

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. (ms) */
#endif

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
    // unit from pa to hpa
    seaLevelHpa = p / IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING / 100;
// temper = t / IMU_BMP_SEA_LEVEL_PRESSURE_SAMPLING;
#endif

    state = IMU_OK;
    return state;
}

#ifdef USE_PERIPHERAL_MPU6050
volatile bool mpuInterrupt =
    false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void IMU::imu_isr_update()
{
    if (!dmpReady)
        return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        // Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
    }
}
#endif

float IMU::altitude_filter(float v)
{
    static float decay = v;
    decay = IMU_ALTITUDE_SMOOTHING_CONSTANT * decay +
            (1 - IMU_ALTITUDE_SMOOTHING_CONSTANT) * v;
    return decay;
}

#ifdef USE_PERIPHERAL_BMP280
/* The criteria to determine launching state:
 * 1. The average first derivative of altitude is larger than
 */
void IMU::bmp_update()
{
    altitude = altitude_filter(bmp.readAltitude(seaLevelHpa));

    static float lastAltitude = altitude;

    // times 1000 because unit changes from ms to s
    // TODO: use third derivative
    float derivative =
        1000 * (altitude - lastAltitude) / IMU_BMP_SAMPLING_PERIOD;

    // Updating altitude record
    lastAltitude = altitude;

    if (derivative > IMU_RISING_CRITERIA) {
        // Rocket rising
        pose = ROCKET_RISING;
    } else if (derivative < IMU_FALLING_CRITERIA) {
        // Rocket falling
        pose = ROCKET_FALLING;
    } else {
        pose = ROCKET_UNKNOWN;
    }
}
#endif