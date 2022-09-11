/**
 * @file mpu6050.h
 * @author Ailton Fidelix (ailton1626@gmail.com)
 * @brief MPU6050 library for ESP-IDF
 * @version 1.0.0
 * @date 07-08-2022
 * @copyright Copyright (c) 2022
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

// I2C master clock
#define MPU6050_I2C_FREQ_HZ 400000 

// Device address
#define MPU6050_ADDRESS 0x69

// Device default address
#define MPU6050_DEFAULT_ADDRESS 0x68

// MPU6050 internal registers addresses used
#define MPU6050_CONFIG 0x1A
#define MPU6050_SMPRT_DIV 0x19
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_FIFO_EN 0x23
#define MPU6050_I2C_MST_CTRL 0x24
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

// MPU6050 band values
#define MPU6050_BAND_260_HZ 0x00
#define MPU6050_BAND_184_HZ 0x01
#define MPU6050_BAND_94_HZ 0x02
#define MPU6050_BAND_44_HZ 0x03
#define MPU6050_BAND_21_HZ 0x04
#define MPU6050_BAND_10_HZ 0x05
#define MPU6050_BAND_5_HZ 0x06

// MPU6050 gyroscope scale ranges
#define MPU6050_GYRO_RANGE_2000DPS 0x18
#define MPU6050_GYRO_RANGE_1000DPS 0x10
#define MPU6050_GYRO_RANGE_500DPS 0x08
#define MPU6050_GYRO_RANGE_250DPS 0x00

// MPU6050 accelerometer scale ranges
#define MPU6050_ACCEL_RANGE_16G 0x18
#define MPU6050_ACCEL_RANGE_8G 0x10
#define MPU6050_ACCEL_RANGE_4G 0x08
#define MPU6050_ACCEL_RANGE_2G 0x00

#define TEMP_FIFO_EN 0x80  // This enables TEMP_OUT_H and TEMP_OUT_L to be written into the FIFO buffer.
#define XG_FIFO_EN 0x40    // This enables GYRO_XOUT_H and GYRO_XOUT_L to be written into the FIFO buffer.
#define YG_FIFO_EN 0x20    // This enables GYRO_YOUT_H and GYRO_YOUT_L to be written into the FIFO buffer.
#define ZG_FIFO_EN 0x10    // This enables GYRO_ZOUT_H and GYRO_ZOUT_L to be written into the FIFO buffer.
#define ACCEL_FIFO_EN 0x08 // This enables all ACCEL_OUT to be written into the FIFO buffer.
#define DISABLE_FIFO 0x00  // This disable the FIFO buffer

#define WRITE_BIT I2C_MASTER_WRITE // I2C master write
#define READ_BIT I2C_MASTER_READ   // I2C master read
#define ACK_CHECK_EN 0x1           // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0          // I2C master will not check ack from slave
#define ACK_VAL 0x0                // I2C ack value
#define NACK_VAL 0x1               // I2C nack value

// Functions used to initialize the communication, write and read
esp_err_t mpuBegin(uint8_t accel_range, uint8_t gyro_range, bool install_driver);
esp_err_t mpuReadSensors();
uint8_t mpuReadByte(uint8_t reg);
esp_err_t mpuWriteByte(uint8_t reg, uint8_t value);

// Functions used to configure the device
esp_err_t mpuSetSleepMode(bool mode);
esp_err_t mpuSetFilterBandwidth(uint8_t bandwidth);
esp_err_t mpuSetFIFObuffer(uint8_t fifo);
esp_err_t mpuSetAccelRange(uint8_t accel_range);
esp_err_t mpuSetGyroRange(uint8_t gyro_range);

// Functions used to get the sensors values
float mpuGetTemperature();
float mpuGetAccelerationX();
float mpuGetAccelerationY();
float mpuGetAccelerationZ();
float mpuGetGyroscopeX();
float mpuGetGyroscopeY();
float mpuGetGyroscopeZ();

#endif