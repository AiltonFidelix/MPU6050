/**
 * @file mpu6050.c
 * @author Ailton Fidelix (ailton1626@gmail.com)
 * @brief MPU6050 library for ESP-IDF
 * @version 1.0.0
 * @date 07-08-2022
 * @copyright Copyright (c) 2022
 */

#include "mpu6050.h"

// Buffer used to read the sensors registers
uint8_t buffer[14];

// accel and gyro sensitivity
float accel_sensitivity;
float gyro_sensitivity;

// TAG used for MPU6050 sensor logi
static const char TAG[] = "mpu6050";

/**
 * @brief Initialize the MPU6050 I2C connection
 * @param accel_range accelerometer range scale
 * @param gyro_range gyroscope range scale
 * @param install_driver if need install the I2C driver
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
 */
esp_err_t mpuBegin(uint8_t accel_range, uint8_t gyro_range, bool install_driver)
{
    ESP_LOGI(TAG, "Beginning connection");

    if (install_driver)
    {
        ESP_LOGI(TAG, "Installing I2C driver");

        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_NUM_21,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = GPIO_NUM_22,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = MPU6050_I2C_FREQ_HZ,
        };

        i2c_param_config(I2C_NUM_0, &i2c_config);

        esp_err_t ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C driver install failed");
            return ret;
        }
        ESP_LOGI(TAG, "I2C driver install success");
    }
    else
    {
        ESP_LOGI(TAG, "I2C driver not installed");
    }

    if (MPU6050_ADDRESS == MPU6050_DEFAULT_ADDRESS)
    {
        if (mpuReadByte(MPU6050_WHO_AM_I) != MPU6050_ADDRESS)
        {
            ESP_LOGE(TAG, "Device with 0x%x address not found", MPU6050_ADDRESS);
            return ESP_ERR_INVALID_ARG;
        }
        else
        {
            ESP_LOGI(TAG, "Device with 0x%x address found", MPU6050_ADDRESS);
        }
    }
    else
    {
        ESP_LOGI(TAG, "Device with 0x%x address, not default", MPU6050_ADDRESS);
    }

    // Configure the gyroscope and accelerometer scale ranges
    mpuSetAccelRange(accel_range);
    mpuSetGyroRange(gyro_range);

    // Disable the FIFO buffer
    mpuSetFIFObuffer(DISABLE_FIFO);

    // Set the sample rate divider to 0
    mpuWriteByte(MPU6050_SMPRT_DIV, 0);
    // Disable multi-master mode and set master clock to 400KHz
    mpuWriteByte(MPU6050_I2C_MST_CTRL, 0x0D);

    // Disable sleep mode
    return mpuSetSleepMode(false);
}

/**
 * @brief Configure the MPU6050 accelerometer range
 * @param accel_range accelerometer range to be set
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
 */
esp_err_t mpuSetAccelRange(uint8_t accel_range)
{
    switch (accel_range)
    {
    case MPU6050_ACCEL_RANGE_2G:
        ESP_LOGI(TAG, "Setting accelerometer range to 2G");
        accel_sensitivity = 16384.0;
        break;
    case MPU6050_ACCEL_RANGE_4G:
        ESP_LOGI(TAG, "Setting accelerometer range to 4G");
        accel_sensitivity = 8192.0;
        break;
    case MPU6050_ACCEL_RANGE_8G:
        ESP_LOGI(TAG, "Setting accelerometer range to 8G");
        accel_sensitivity = 4096.0;
        break;
    case MPU6050_ACCEL_RANGE_16G:
        ESP_LOGI(TAG, "Setting accelerometer range to 16G");
        accel_sensitivity = 2048.0;
        break;
    }

    return mpuWriteByte(MPU6050_ACCEL_CONFIG, accel_range);
}

/**
 * @brief Configure the MPU6050 gyroscope range
 * @param gyro_range gyroscope range to be set
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
 */
esp_err_t mpuSetGyroRange(uint8_t gyro_range)
{
    switch (gyro_range)
    {
    case MPU6050_GYRO_RANGE_250DPS:
        ESP_LOGI(TAG, "Setting gyroscope range to 250DPS");
        gyro_sensitivity = 131.0;
        break;
    case MPU6050_GYRO_RANGE_500DPS:
        ESP_LOGI(TAG, "Setting gyroscope range to 500DPS");
        gyro_sensitivity = 65.5;
        break;
    case MPU6050_GYRO_RANGE_1000DPS:
        ESP_LOGI(TAG, "Setting gyroscope range to 1000DPS");
        gyro_sensitivity = 32.8;
        break;
    case MPU6050_GYRO_RANGE_2000DPS:
        ESP_LOGI(TAG, "Setting gyroscope range to 2000DPS");
        gyro_sensitivity = 16.4;
        break;
    }

    return mpuWriteByte(MPU6050_GYRO_CONFIG, gyro_range);
}

/**
 * @brief Configure the MPU6050 sleep mode
 * @param mode true to enable or false to disable
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
 */
esp_err_t mpuSetSleepMode(bool mode)
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

    return mpuWriteByte(MPU6050_PWR_MGMT_1, mode);
}

/**
 * @brief Configure the accelerometer and gyroscope filter
 * @param bandwidth Filter value to be set
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
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
    return mpuWriteByte(MPU6050_CONFIG, bandwidth);
}

/**
 * @brief Configure the FIFO buffer
 * @param fifo mode that FIFO will operate
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
 */
esp_err_t mpuSetFIFObuffer(uint8_t fifo)
{
    switch (fifo)
    {
    case DISABLE_FIFO:
        ESP_LOGI(TAG, "FIFO buffer was disabled");
        break;
    case TEMP_FIFO_EN:
        ESP_LOGI(TAG, "Temperature FIFO buffer was enabled");
        break;
    case XG_FIFO_EN:
        ESP_LOGI(TAG, "Gyroscope X FIFO buffer was enabled");
        break;
    case YG_FIFO_EN:
        ESP_LOGI(TAG, "Gyroscope Y FIFO buffer was enabled");
        break;
    case ZG_FIFO_EN:
        ESP_LOGI(TAG, "Gyroscope Z FIFO buffer was enabled");
        break;
    case ACCEL_FIFO_EN:
        ESP_LOGI(TAG, "Accelerometer FIFO buffer was enabled");
        break;
    }
    return mpuWriteByte(MPU6050_FIFO_EN, fifo);
}

/**
 * @brief Get the temperature
 * @return float the temperature value in Celsius
 */
float mpuGetTemperature()
{
    int16_t result = (((int16_t)buffer[6]) << 8) | buffer[7];
    return (result / 340.0) + 36.53;
}

/**
 * @brief Get the X acceleration
 * @return float X acceleration value in the range configured
 */
float mpuGetAccelerationX()
{
    int16_t result = (((int16_t)buffer[0]) << 8) | buffer[1];
    return result / accel_sensitivity;
}

/**
 * @brief Get the Y acceleration
 * @return float Y acceleration value in the range configured
 */
float mpuGetAccelerationY()
{
    int16_t result = (((int16_t)buffer[2]) << 8) | buffer[3];
    return result / accel_sensitivity;
}

/**
 * @brief Get the Z acceleration
 * @return float Z acceleration value in the range configured
 */
float mpuGetAccelerationZ()
{
    int16_t result = (((int16_t)buffer[4]) << 8) | buffer[5];
    return result / accel_sensitivity;
}

/**
 * @brief Get the X gyroscope
 * @return float X gyroscope value in the range configured
 */
float mpuGetGyroscopeX()
{
    int16_t result = (((int16_t)buffer[8]) << 8) | buffer[9];
    return result / gyro_sensitivity;
}

/**
 * @brief Get the Y gyroscope
 * @return float Y gyroscope value in the range configured
 */
float mpuGetGyroscopeY()
{
    int16_t result = (((int16_t)buffer[10]) << 8) | buffer[11];
    return result / gyro_sensitivity;
}

/**
 * @brief Get the Z gyroscope
 * @return float Z gyroscope value in the range configured
 */
float mpuGetGyroscopeZ()
{
    int16_t result = (((int16_t)buffer[12]) << 8) | buffer[13];
    return result / gyro_sensitivity;
}

/**
 * @brief Read MPU6050 sensors registers 0x3B ~ 0x47
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
 */
esp_err_t mpuReadSensors()
{
    ESP_LOGI(TAG, "Reading sensors registers");

    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);

    // Read the registers
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
uint8_t mpuReadByte(uint8_t reg)
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
 * @return esp_err_t ESP_OK if success, otherwise ESP_FAIL
 */
esp_err_t mpuWriteByte(uint8_t reg, uint8_t value)
{
    ESP_LOGI(TAG, "Writing in the 0x%x register the value 0x%x", reg, value);

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