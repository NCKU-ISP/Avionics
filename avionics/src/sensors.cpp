#include "sensors.h"
#include "Arduino.h"

#include "configs.h"

// MPU 9250
#include "MPU9250.h"

IMU::IMU(){
    state = IMU_ERROR;

    // Failed to initialize MPU9250
    if ( mpu.begin() < 0 ) return;

    // Failed to initialize BMP280
    if ( !bmp.begin() ) return;

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

    // Interrupt setting
    pinMode(PIN_IMU_INT, INPUT);
    attachInterrupt(PIN_IMU_INT, imu_isr_update, RISING);


    /* BMP 280 settings */
    bmp.setOversampling(IMU_BMP_OVERSAMPLE);
    state = IMU_OK;
}

IMU_STATE IMU::init(){
    state = IMU_ERROR;
    // Get sea level information
    char delay_time = bmp.startMeasurment();
    if ( delay_time!= 0 ){
        delay(delay_time);
        if ( bmp.getTemperatureAndPressure(sea_level_pressure, temper) == 0)
            return state;   // Error 
    }
    // Return with error message
    else return state;

    state = IMU_OK;
    return state;
}

void IMU::acceleration_filter(){

}

void IMU::gyro_filter(){

}

double IMU::pressure_filter(double pressure){
    // Note, already applied digital low pass filter with its library
}

void IMU::imu_isr_update(){
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

void IMU::bmp_update(){
    static bool bmp_reading = false;
    static unsigned long next_reading_time = 0; 
    if ( !bmp_reading ){
        // startMeasurement will return the time(ms) required to wait
        next_reading_time = millis() + bmp.startMeasurment();
        bmp_reading = true;
    }
    else if ( millis() > next_reading_time){
        // bmp reading and 
        if ( bmp.getTemperatureAndPressure(temper, pressure) == 0 )
            state = IMU_ERROR;
        altitude = bmp.altitude(pressure, sea_level_pressure);
        bmp_reading = false;
    }
}