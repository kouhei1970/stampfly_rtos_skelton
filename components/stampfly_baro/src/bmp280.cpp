/**
 * @file bmp280.cpp
 * @brief BMP280 Barometer Driver Implementation
 */

#include "bmp280.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <cmath>

static const char* TAG = "BMP280";

namespace stampfly {

BMP280::BMP280()
    : i2c_port_(I2C_NUM_0)
    , i2c_addr_(0x76)
    , mutex_(nullptr)
    , initialized_(false)
    , sea_level_pressure_(1013.25f)
    , reference_altitude_(0.0f)
    , t_fine_(0)
    , calib_{} {
}

BMP280::~BMP280() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t BMP280::init(const Config& config) {
    esp_err_t ret;

    i2c_port_ = config.i2c_port;
    i2c_addr_ = config.i2c_addr;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Soft reset
    ret = softReset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify chip ID
    uint8_t chip_id = getChipId();
    if (chip_id != CHIP_ID_BMP280 && chip_id != CHIP_ID_BME280) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X", chip_id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMP280/BME280 detected, chip ID: 0x%02X", chip_id);

    // Read calibration data
    ret = readCalibration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    // Configure sensor
    // CONFIG: standby = 0.5ms, filter coeff = 16
    ret = writeRegister(REG_CONFIG, 0x10);  // Filter x16
    if (ret != ESP_OK) return ret;

    // CTRL_MEAS: osrs_t = x2, osrs_p = x16, mode = normal
    ret = writeRegister(REG_CTRL_MEAS, 0x57);  // Temp x2, Press x16, Normal mode
    if (ret != ESP_OK) return ret;

    initialized_ = true;
    ESP_LOGI(TAG, "BMP280 initialized at address 0x%02X", i2c_addr_);
    return ESP_OK;
}

esp_err_t BMP280::readCalibration() {
    uint8_t data[24];
    esp_err_t ret = readRegisters(REG_CALIB_00, data, 24);
    if (ret != ESP_OK) return ret;

    calib_.dig_T1 = (uint16_t)(data[1] << 8) | data[0];
    calib_.dig_T2 = (int16_t)((data[3] << 8) | data[2]);
    calib_.dig_T3 = (int16_t)((data[5] << 8) | data[4]);
    calib_.dig_P1 = (uint16_t)(data[7] << 8) | data[6];
    calib_.dig_P2 = (int16_t)((data[9] << 8) | data[8]);
    calib_.dig_P3 = (int16_t)((data[11] << 8) | data[10]);
    calib_.dig_P4 = (int16_t)((data[13] << 8) | data[12]);
    calib_.dig_P5 = (int16_t)((data[15] << 8) | data[14]);
    calib_.dig_P6 = (int16_t)((data[17] << 8) | data[16]);
    calib_.dig_P7 = (int16_t)((data[19] << 8) | data[18]);
    calib_.dig_P8 = (int16_t)((data[21] << 8) | data[20]);
    calib_.dig_P9 = (int16_t)((data[23] << 8) | data[22]);

    return ESP_OK;
}

esp_err_t BMP280::read(BaroData& data) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read 6 bytes: press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb
    uint8_t buffer[6];
    esp_err_t ret = readRegisters(REG_PRESS_MSB, buffer, 6);

    xSemaphoreGive(mutex_);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse raw values (20-bit)
    int32_t adc_P = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | ((int32_t)buffer[2] >> 4);
    int32_t adc_T = ((int32_t)buffer[3] << 12) | ((int32_t)buffer[4] << 4) | ((int32_t)buffer[5] >> 4);

    // Compensate temperature first (sets t_fine)
    int32_t temp_comp = compensateTemperature(adc_T);
    data.temperature = (float)temp_comp / 100.0f;

    // Compensate pressure
    uint32_t press_comp = compensatePressure(adc_P);
    data.pressure = (float)press_comp / 256.0f / 100.0f;  // Convert Pa to hPa

    // Calculate altitude
    data.altitude = calculateAltitude(data.pressure) - reference_altitude_;

    data.timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

int32_t BMP280::compensateTemperature(int32_t adc_T) {
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)calib_.dig_T1 << 1))) * ((int32_t)calib_.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_.dig_T1))) >> 12) *
            ((int32_t)calib_.dig_T3)) >> 14;

    t_fine_ = var1 + var2;
    T = (t_fine_ * 5 + 128) >> 8;

    return T;  // Temperature in 0.01 degrees C
}

uint32_t BMP280::compensatePressure(int32_t adc_P) {
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine_) - 128000;
    var2 = var1 * var1 * (int64_t)calib_.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_.dig_P3) >> 8) + ((var1 * (int64_t)calib_.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_.dig_P7) << 4);

    return (uint32_t)p;  // Pressure in Q24.8 format (Pa * 256)
}

float BMP280::calculateAltitude(float pressure) {
    // Barometric formula
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure_, 0.1903f));
}

void BMP280::setSeaLevelPressure(float pressure_hpa) {
    sea_level_pressure_ = pressure_hpa;
}

void BMP280::setReferenceAltitude(float altitude_m) {
    reference_altitude_ = altitude_m;
}

bool BMP280::isDataReady() {
    uint8_t status;
    if (readRegister(REG_STATUS, &status) == ESP_OK) {
        return (status & 0x08) == 0;  // measuring bit = 0 means data ready
    }
    return false;
}

uint8_t BMP280::getChipId() {
    uint8_t id = 0;
    readRegister(REG_CHIP_ID, &id);
    return id;
}

esp_err_t BMP280::softReset() {
    return writeRegister(REG_SOFT_RESET, CMD_SOFT_RESET);
}

esp_err_t BMP280::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t BMP280::readRegister(uint8_t reg, uint8_t* value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t BMP280::readRegisters(uint8_t reg, uint8_t* buffer, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

}  // namespace stampfly
