/**
 * @file mpu6050.c
 * @author Ailton Fidelix (ailton1626@gmail.com)
 * @brief MPU6050 library for ESP-IDF
 * @version 0.1
 * @date 2022-07-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "mpu6050.h"

// Buffer used to read sensors registers
uint8_t buffer[14];

// TAG used for MPU6050 sensor logi
static const char TAG[] = "mpu6050";

/**
 * @brief Initialize the MPU6050 I2C connection
 * @return esp_err_t ESP_OK if connection was ok, otherwise ESP_ERR
 */
esp_err_t mpuBegin()
{
    ESP_LOGI(TAG, "Beginning connection");

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    i2c_param_config(I2C_NUM_0, &i2c_config);

    esp_err_t ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    if (ret != ESP_OK)
        return ret;

    if (mpuReadSingleRegister(MPU6050_RA_WHO_AM_I) != MPU6050_ADDRESS)
    {
        ESP_LOGI(TAG, "Device with 0x%x address not found", MPU6050_ADDRESS);
        return ESP_ERR_INVALID_ARG;
    }
    else
    {
        ESP_LOGI(TAG, "Device with 0x%x address found", MPU6050_ADDRESS);
    }

    return setSleepMode(true);
}

/**
 * @brief Wake up MPU6050 device
 * @param bool mode enable or disable
 * @return esp_err_t ESP_OK if the sleep mode set was ok, otherwise ESP_ERR
 */
esp_err_t setSleepMode(bool mode)
{
    switch (mode)
    {
    case true:
        ESP_LOGI(TAG, "Enable device sleep mode");
        break;
    case false:
        ESP_LOGI(TAG, "Disable device sleep mode");
        break;
    }

    return mpuWriteSingleRegister(MPU6050_PWR_MGMT_1, mode);
}

/**
 * @brief Configure the accelerometer and gyroscope filter
 * @param bandwidth Receive the filter value
 * @return esp_err_t ESP_OK if the bandwidth set was ok, otherwise ESP_ERR
 */
esp_err_t mpuSetFilterBandwidth(uint8_t bandwidth)
{
    switch (bandwidth)
    {
    case MPU6050_BAND_5_HZ:
        ESP_LOGI(TAG, "Setting Bandwidth 5Hz");
        break;
    case MPU6050_BAND_10_HZ:
        ESP_LOGI(TAG, "Setting Bandwidth 10Hz");
        break;
    case MPU6050_BAND_21_HZ:
        ESP_LOGI(TAG, "Setting Bandwidth 21Hz");
        break;
    case MPU6050_BAND_44_HZ:
        ESP_LOGI(TAG, "Setting Bandwidth 44Hz");
        break;
    case MPU6050_BAND_94_HZ:
        ESP_LOGI(TAG, "Setting Bandwidth 94Hz");
        break;
    case MPU6050_BAND_184_HZ:
        ESP_LOGI(TAG, "Setting Bandwidth 184Hz");
        break;
    case MPU6050_BAND_260_HZ:
        ESP_LOGI(TAG, "Setting Bandwidth 260Hz");
        break;
    default:
        break;
    }
    return mpuWriteSingleRegister(MPU6050_CONFIG, bandwidth);
}

/**
 * @brief Get the temperature in Celsius
 * @return float 
 */
float mpuReadTemperature()
{
    int16_t result = buffer[6] << 8 | buffer[7];
    return (result / 340.0) + 36.53;
}

/**
 * @brief Get the X acceleration
 * @return float
 */
float mpuReadAccelerationX()
{
    int16_t result = ((buffer[0] << 0) & 0xFF) | ((int16_t)(buffer[1] << 8));
    return result / 16384.0;
}

/**
 * @brief Get the Y acceleration
 * @return float
 */
float mpuReadAccelerationY()
{
    int16_t result = ((buffer[2] << 0) & 0xFF) | ((int16_t)(buffer[3] << 8));
    return result / 16384.0;
}

/**
 * @brief Get the Z acceleration
 * @return float
 */
float mpuReadAccelerationZ()
{
    int16_t result = buffer[4] << 8 | buffer[5];
    return result / 16384.0;
}

/**
 * @brief Get the X gyroscope
 * @return float
 */
float mpuReadGyroscopeX()
{
    int16_t result = buffer[8] << 8 | buffer[9];
    return result / 16384.0;
}

/**
 * @brief Get the Y gyroscope
 * @return float
 */
float mpuReadGyroscopeY()
{
    int16_t result = buffer[10] << 8 | buffer[11];
    return result / 16384.0;
}

/**
 * @brief Get the Z gyroscope
 * @return float
 */
float mpuReadGyroscopeZ()
{
    int16_t result = buffer[12] << 8 | buffer[13];
    return result / 16384.0;
}

/**
 * @brief Read MPU6050 sensors registers
 * @return esp_err_t ESP_OK if the reads were ok, otherwise ESP_ERR
 */
esp_err_t mpuReadSensors()
{
    ESP_LOGI(TAG, "Reading sensors registers");

    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_RA_ACCEL_XOUT_H, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);

    // Read all registers
    uint8_t i;
    for (i = 0; i < (sizeof(buffer) / sizeof(buffer[0])) - 1; i++)
    {
        i2c_master_read_byte(cmd, &buffer[i], ACK_VAL);
    }
    i2c_master_read_byte(cmd, &buffer[i], NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief Read a single register from MPU6050 device
 * @param reg the register to be read
 * @return uint8_t value that was read
 */
uint8_t mpuReadSingleRegister(uint8_t reg)
{
    ESP_LOGI(TAG, "Reading the 0x%x register", reg);

    uint8_t value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &value, ACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return value;
}

/**
 * @brief Write a single register in the MPU6050 device
 * @param reg the register to be write
 * @param value to be write
 * @return esp_err_t ESP_OK if the write was ok, otherwise ESP_ERR
 */
esp_err_t mpuWriteSingleRegister(uint8_t reg, uint8_t value)
{
    ESP_LOGI(TAG, "Writing in the 0x%x register the value %d", reg, value);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}