# 國立成功大學 太空推進研究社
National Cheng Kung University Institute of Space Propulsion (NCKU ISP)

## Introduction
We are a group of students interested in space technology, and we established the student club in NCKU in Sep. 2020. 

## Avionics System 航電系統
These programs are still under developing. For different applications, we designed several versions of the avionic system, they are:
- Medium 
  This version provides basic peripheral equipment connecting abilities and equipped with basic parachute releasing functions. This version should be adequate for general small rocket usage.
- Pro
  We developed this version for an advanced controlled rocket plan, which requires much more computational resources and more precise sensor feedback, and needs the communication capability.

### Medium Version
#### System Structure 系統架構
The MCU: Atmega 328p
Developing Framework: Arduino

The supporting (optional) sensors or peripheral equipments:
- Pressure Sensor(I2C): BMP280 module, the system calculates the parachute releasing timing by differentiating the altitude.
- IMU(I2C): MPU6050, The program includes the digital motion processor (DMP) library from [jrowberg/i2cdevlib](https://github.com/jrowberg/i2cdevlib)
- SD(SPI): One can use a SD card to record all the sensor or the flight information, please refer to the logger module.

#### Circuit Design
![](https://i.imgur.com/jf3Xtqu.png)
![](https://i.imgur.com/mI2GNRR.png)

# Contact Us
To report any bugs or commit any new features, please create a new pull request and describe the issues in detail.
If you are interested in our project and want to support us, please contact us with the following email: `e94066157@gs.ncku.edu.tw`, thanks!
