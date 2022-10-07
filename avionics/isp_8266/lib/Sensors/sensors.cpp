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
    :
#ifdef USE_GY91_MPU9250
      mpu(Wire, 0x68),
#endif
#ifdef USE_GPS_NEO6M
      gpsSerial(PIN_GPS_RX, PIN_GPS_TX), gpsCode(""),
#endif
      used(true) // Check if it is initialized and macro arragement
{
  pose = ROCKET_UNKNOWN;
}

ERROR_CODE IMU::init() {
#ifdef USE_PERIPHERAL_BMP280
  altitudeKalmanFilter = new SimpleKalmanFilter(1, 1, 0.01);
#endif

#ifdef USE_PERIPHERAL_MPU6050
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having
                         // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(PIN_IMU_INT, INPUT);

  if (mpu.testConnection())
    return ERROR_MPU_INIT_FAILED;

  // load and configure the DMP
  // accel scale default to +/- 2g
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
  mpu.setXAccelOffset(-2424);
  mpu.setYAccelOffset(879);
  mpu.setZAccelOffset(1064);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
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
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(devStatus);
    // Serial.println(F(")"));
    return ERROR_DMP_INIT_FAILED;
  }
#elif defined(USE_GY91_MPU9250)
  int status = mpu.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    return ERROR_MPU_INIT_FAILED;
  }
  // setting the accelerometer full scale range to +/-8G
  mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  mpu.setSrd(19);
#endif

#ifdef USE_PERIPHERAL_BMP280
  bmp.reset();
  /* BMP 280 settings */
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))
    return ERROR_BMP_INIT_FAILED;

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. (ms) */
#endif

#ifdef USE_GPS_NEO6M
  gpsSerial.begin(SERIAL_COMMS_BAUDRATE);
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
// altitudeKalmanFilter.setEstimateError(bmp.readAltitude(seaLevelHpa));
#endif
  return ERROR_OK;
}

/*
volatile bool mpuInterrupt =
    false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}*/

#if defined(USE_PERIPHERAL_MPU6050) || defined(USE_GY91_MPU9250)
bool IMU::imu_isr_update() {
#ifdef USE_PERIPHERAL_MPU6050
  if (dmpReady &&
      mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    /*
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);*/
    return true;
  }
  return false;
#elif defined(USE_GY91_MPU9250)
  if (mpu.readSensor() != 1)
    return false;
  return true;
#endif
}
#endif

float IMU::altitude_filter(float v) {
  static float decay = v;
  decay = IMU_ALTITUDE_SMOOTHING_CONSTANT * decay +
          (1 - IMU_ALTITUDE_SMOOTHING_CONSTANT) * v;
  return decay;
}

/* The criteria to determine launching state:
 * 1. The average first derivative of altitude is larger than
 */
float IMU::bmp_update() {
#ifdef USE_PERIPHERAL_BMP280
// altitude = altitude_filter(bmp.readAltitude(seaLevelHpa));
#ifdef USE_PERIPHERAL_BMP280
  static float lastAltitude = est_altitude;
  float bmp_read_altitude = bmp.readAltitude(seaLevelHpa);

  if (bmp_read_altitude != altitude) {
    altitude = bmp.readAltitude(seaLevelHpa);
    est_altitude = altitudeKalmanFilter->updateEstimate(altitude);

    // times 1000 because unit changes from ms to s
    // TODO: use third derivative
    velocity = 1000 * (est_altitude - lastAltitude) / IMU_BMP_SAMPLING_PERIOD;

    // Updating altitude record
    lastAltitude = est_altitude;

    // static auto t = micros();
    // auto tv = micros() - t;
    // t = micros();
    // Serial.println(tv);
  }
#endif

  if (velocity > IMU_RISING_CRITERIA) {
    // Rocket rising
    pose = ROCKET_RISING;
  } else if (velocity < IMU_FALLING_CRITERIA) {
    // Rocket falling
    pose = ROCKET_FALLING;
  } else {
    pose = ROCKET_UNKNOWN;
  }
#endif
  return est_altitude;
}
