/**
 * @file mpu6050.h
 * @author Ailton Fidelix (ailton1626@gmail.com)
 * @brief MPU6050 library for ESP-IDF
 * @version 0.1
 * @date 2022-07-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

#define I2C_FREQ_HZ 400000

#define MPU6050_ADDRESS 0x68 // device address

// MPU6050 internal registers
#define MPU6050_CONFIG 0x1A
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_RA_WHO_AM_I 0x75

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

// MPU6050 band values
#define MPU6050_BAND_260_HZ 0x00
#define MPU6050_BAND_184_HZ 0x01
#define MPU6050_BAND_94_HZ 0x02
#define MPU6050_BAND_44_HZ 0x03
#define MPU6050_BAND_21_HZ 0x04
#define MPU6050_BAND_10_HZ 0x05
#define MPU6050_BAND_5_HZ 0x06

esp_err_t mpuBegin();
esp_err_t mpuReadSensors();
uint8_t mpuReadSingleRegister(uint8_t reg);
esp_err_t mpuWriteSingleRegister(uint8_t reg, uint8_t value);

esp_err_t setSleepMode(bool mode);
esp_err_t mpuSetFilterBandwidth(uint8_t bandwidth);

// Reading sensor values functions
float mpuReadTemperature();
float mpuReadAccelerationX();
float mpuReadAccelerationY();
float mpuReadAccelerationZ();
float mpuReadGyroscopeX();
float mpuReadGyroscopeY();
float mpuReadGyroscopeZ();

#endif