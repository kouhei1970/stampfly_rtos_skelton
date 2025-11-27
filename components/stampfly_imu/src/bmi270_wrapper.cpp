/**
 * @file bmi270_wrapper.cpp
 * @brief BMI270 IMU Driver Implementation
 */

#include "bmi270_wrapper.hpp"
#include "bmi270_config.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

static const char* TAG = "BMI270";

namespace stampfly {

BMI270::BMI270()
    : spi_handle_(nullptr)
    , int1_pin_(GPIO_NUM_NC)
    , config_{}
    , calibration_{}
    , mutex_(nullptr)
    , initialized_(false) {
}

BMI270::~BMI270() {
    if (spi_handle_) {
        spi_bus_remove_device(spi_handle_);
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t BMI270::init(const Config& config) {
    esp_err_t ret;
    config_ = config;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_config = {};
    dev_config.command_bits = 0;
    dev_config.address_bits = 0;
    dev_config.dummy_bits = 0;
    dev_config.mode = 0;  // CPOL=0, CPHA=0
    dev_config.duty_cycle_pos = 128;
    dev_config.cs_ena_pretrans = 0;
    dev_config.cs_ena_posttrans = 0;
    dev_config.clock_speed_hz = config.clock_speed_hz;
    dev_config.input_delay_ns = 0;
    dev_config.spics_io_num = config.cs_pin;
    dev_config.flags = 0;
    dev_config.queue_size = 1;
    dev_config.pre_cb = nullptr;
    dev_config.post_cb = nullptr;

    ret = spi_bus_add_device(config.spi_host, &dev_config, &spi_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure INT1 pin if specified
    int1_pin_ = config.int1_pin;
    if (int1_pin_ != GPIO_NUM_NC) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << int1_pin_);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
    }

    // Perform soft reset
    ret = softReset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed");
        return ret;
    }

    // Wait for boot (spec says 2ms)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Dummy read to switch to SPI mode
    uint8_t dummy;
    readRegister(REG_CHIP_ID, &dummy);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Verify chip ID
    uint8_t chip_id = getChipId();
    if (chip_id != CHIP_ID_BMI270) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, CHIP_ID_BMI270);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMI270 detected, chip ID: 0x%02X", chip_id);

    // Upload config file
    ret = uploadConfigFile();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config file upload failed");
        return ret;
    }

    // Configure sensor
    ret = configureSensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor configuration failed");
        return ret;
    }

    // Initialize calibration to zero
    memset(&calibration_, 0, sizeof(calibration_));

    initialized_ = true;
    ESP_LOGI(TAG, "BMI270 initialized successfully");
    return ESP_OK;
}

esp_err_t BMI270::uploadConfigFile() {
    esp_err_t ret;

    // Disable advanced power save mode for config upload
    ret = writeRegister(REG_PWR_CONF, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Prepare for config load
    ret = writeRegister(REG_INIT_CTRL, 0x00);
    if (ret != ESP_OK) return ret;

    // Upload config file in bursts
    const uint16_t chunk_size = 64;
    for (uint16_t i = 0; i < sizeof(bmi270_config_file); i += chunk_size) {
        uint16_t remaining = sizeof(bmi270_config_file) - i;
        uint16_t len = (remaining < chunk_size) ? remaining : chunk_size;

        // Set burst write address
        uint8_t addr_low = (i / 2) & 0x0F;
        uint8_t addr_high = (i / 2) >> 4;

        // Write index
        ret = writeRegister(0x5B, addr_low);  // INIT_ADDR_0
        if (ret != ESP_OK) return ret;
        ret = writeRegister(0x5C, addr_high); // INIT_ADDR_1
        if (ret != ESP_OK) return ret;

        // Write data burst
        uint8_t tx_buf[len + 1];
        tx_buf[0] = REG_INIT_DATA;
        memcpy(&tx_buf[1], &bmi270_config_file[i], len);

        spi_transaction_t trans = {};
        trans.length = (len + 1) * 8;
        trans.tx_buffer = tx_buf;
        trans.rx_buffer = nullptr;

        ret = spi_device_polling_transmit(spi_handle_, &trans);
        if (ret != ESP_OK) return ret;
    }

    // Complete config load
    ret = writeRegister(REG_INIT_CTRL, 0x01);
    if (ret != ESP_OK) return ret;

    // Wait for initialization (spec says 150ms max)
    vTaskDelay(pdMS_TO_TICKS(150));

    // Check internal status
    uint8_t status;
    ret = readRegister(REG_INTERNAL_STATUS, &status);
    if (ret != ESP_OK) return ret;

    if ((status & 0x0F) != 0x01) {
        ESP_LOGE(TAG, "Config upload failed, internal status: 0x%02X", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Config file uploaded successfully");
    return ESP_OK;
}

esp_err_t BMI270::configureSensor() {
    esp_err_t ret;

    // Configure accelerometer
    // ODR = 1600Hz (0x0C), filter normal mode (0x02), OSR4 averaging (0x00)
    // ACC_CONF: odr[3:0]=0xC (1600Hz), bwp[6:4]=0x2 (normal), filter_perf=1
    uint8_t acc_conf = 0x0C | (0x02 << 4) | (0x01 << 7);
    if (config_.sample_rate_hz <= 400) {
        acc_conf = 0x0B | (0x02 << 4) | (0x01 << 7);  // 800Hz ODR
    }
    ret = writeRegister(REG_ACC_CONF, acc_conf);
    if (ret != ESP_OK) return ret;

    // Accelerometer range: ±16g (0x03)
    ret = writeRegister(REG_ACC_RANGE, 0x03);
    if (ret != ESP_OK) return ret;

    // Configure gyroscope
    // ODR = 1600Hz (0x0C), filter normal mode (0x02), noise_perf=1, filter_perf=1
    uint8_t gyr_conf = 0x0C | (0x02 << 4) | (0x01 << 6) | (0x01 << 7);
    if (config_.sample_rate_hz <= 400) {
        gyr_conf = 0x0B | (0x02 << 4) | (0x01 << 6) | (0x01 << 7);  // 800Hz ODR
    }
    ret = writeRegister(REG_GYR_CONF, gyr_conf);
    if (ret != ESP_OK) return ret;

    // Gyroscope range: ±2000dps (0x00)
    ret = writeRegister(REG_GYR_RANGE, 0x00);
    if (ret != ESP_OK) return ret;

    // Configure INT1: active high, push-pull, output enable
    ret = writeRegister(REG_INT1_IO_CTRL, 0x0A);
    if (ret != ESP_OK) return ret;

    // Map data ready interrupt to INT1
    ret = writeRegister(REG_INT_MAP_DATA, 0x04);
    if (ret != ESP_OK) return ret;

    // Enable accelerometer and gyroscope
    ret = writeRegister(REG_PWR_CTRL, 0x0E);  // acc_en=1, gyr_en=1, temp_en=1
    if (ret != ESP_OK) return ret;

    // Set normal power mode
    ret = writeRegister(REG_PWR_CONF, 0x00);
    if (ret != ESP_OK) return ret;

    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Sensor configured: ACC ±16g, GYRO ±2000dps");
    return ESP_OK;
}

esp_err_t BMI270::read(ImuData& data) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[12];

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read accel and gyro data (12 bytes starting from REG_DATA_8)
    esp_err_t ret = burstRead(REG_DATA_8, buffer, 12);

    xSemaphoreGive(mutex_);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse accelerometer data (little endian)
    int16_t acc_raw[3];
    acc_raw[0] = (int16_t)(buffer[0] | (buffer[1] << 8));
    acc_raw[1] = (int16_t)(buffer[2] | (buffer[3] << 8));
    acc_raw[2] = (int16_t)(buffer[4] | (buffer[5] << 8));

    // Parse gyroscope data (little endian)
    int16_t gyr_raw[3];
    gyr_raw[0] = (int16_t)(buffer[6] | (buffer[7] << 8));
    gyr_raw[1] = (int16_t)(buffer[8] | (buffer[9] << 8));
    gyr_raw[2] = (int16_t)(buffer[10] | (buffer[11] << 8));

    // Convert to physical units and apply calibration
    data.accel_x = acc_raw[0] * ACCEL_SCALE_16G - calibration_.accel_bias[0];
    data.accel_y = acc_raw[1] * ACCEL_SCALE_16G - calibration_.accel_bias[1];
    data.accel_z = acc_raw[2] * ACCEL_SCALE_16G - calibration_.accel_bias[2];

    data.gyro_x = gyr_raw[0] * GYRO_SCALE_2000DPS - calibration_.gyro_bias[0];
    data.gyro_y = gyr_raw[1] * GYRO_SCALE_2000DPS - calibration_.gyro_bias[1];
    data.gyro_z = gyr_raw[2] * GYRO_SCALE_2000DPS - calibration_.gyro_bias[2];

    data.timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

esp_err_t BMI270::calibrateGyro(uint16_t num_samples) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting gyro calibration with %d samples...", num_samples);

    float sum[3] = {0, 0, 0};
    ImuData data;

    // Temporarily clear calibration
    CalibrationData old_cal = calibration_;
    memset(&calibration_, 0, sizeof(calibration_));

    for (uint16_t i = 0; i < num_samples; i++) {
        esp_err_t ret = read(data);
        if (ret != ESP_OK) {
            calibration_ = old_cal;
            return ret;
        }
        sum[0] += data.gyro_x;
        sum[1] += data.gyro_y;
        sum[2] += data.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(2));  // ~500Hz sampling during calibration
    }

    calibration_.gyro_bias[0] = sum[0] / num_samples;
    calibration_.gyro_bias[1] = sum[1] / num_samples;
    calibration_.gyro_bias[2] = sum[2] / num_samples;

    ESP_LOGI(TAG, "Gyro bias: X=%.4f, Y=%.4f, Z=%.4f rad/s",
             calibration_.gyro_bias[0],
             calibration_.gyro_bias[1],
             calibration_.gyro_bias[2]);

    return ESP_OK;
}

void BMI270::setCalibration(const CalibrationData& cal) {
    calibration_ = cal;
}

BMI270::CalibrationData BMI270::getCalibration() const {
    return calibration_;
}

bool BMI270::isDataReady() {
    if (int1_pin_ != GPIO_NUM_NC) {
        return gpio_get_level(int1_pin_) == 1;
    }

    // Fallback: read status register
    uint8_t status;
    if (readRegister(REG_INT_STATUS_1, &status) == ESP_OK) {
        return (status & 0x80) != 0;  // acc_drdy and gyr_drdy
    }
    return false;
}

uint8_t BMI270::getChipId() {
    uint8_t chip_id = 0;
    readRegister(REG_CHIP_ID, &chip_id);
    return chip_id;
}

esp_err_t BMI270::softReset() {
    return writeRegister(REG_CMD, CMD_SOFT_RESET);
}

esp_err_t BMI270::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = {reg, value};

    spi_transaction_t trans = {};
    trans.length = 16;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = nullptr;

    return spi_device_polling_transmit(spi_handle_, &trans);
}

esp_err_t BMI270::readRegister(uint8_t reg, uint8_t* value) {
    uint8_t tx_buf[3] = {static_cast<uint8_t>(reg | 0x80), 0x00, 0x00};
    uint8_t rx_buf[3] = {0};

    spi_transaction_t trans = {};
    trans.length = 24;  // 8 bits cmd + 8 bits dummy + 8 bits data
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    esp_err_t ret = spi_device_polling_transmit(spi_handle_, &trans);
    if (ret == ESP_OK) {
        *value = rx_buf[2];  // Data after dummy byte
    }
    return ret;
}

esp_err_t BMI270::readRegisters(uint8_t reg, uint8_t* buffer, size_t len) {
    return burstRead(reg, buffer, len);
}

esp_err_t BMI270::burstRead(uint8_t reg, uint8_t* buffer, size_t len) {
    // BMI270 SPI read: 1 byte address + 1 byte dummy + N data bytes
    uint8_t tx_buf[len + 2];
    uint8_t rx_buf[len + 2];

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = reg | 0x80;  // Read bit

    spi_transaction_t trans = {};
    trans.length = (len + 2) * 8;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    esp_err_t ret = spi_device_polling_transmit(spi_handle_, &trans);
    if (ret == ESP_OK) {
        memcpy(buffer, &rx_buf[2], len);  // Skip address and dummy byte
    }
    return ret;
}

}  // namespace stampfly
