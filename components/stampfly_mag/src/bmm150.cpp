/**
 * @file bmm150.cpp
 * @brief BMM150 Magnetometer Driver Implementation
 */

#include "bmm150.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <cmath>

static const char* TAG = "BMM150";

namespace stampfly {

BMM150::BMM150()
    : i2c_port_(I2C_NUM_0)
    , i2c_addr_(0x10)
    , mutex_(nullptr)
    , initialized_(false)
    , calibration_{}
    , trim_{} {
    // Initialize soft iron matrix to identity
    calibration_.soft_iron[0] = 1.0f;
    calibration_.soft_iron[4] = 1.0f;
    calibration_.soft_iron[8] = 1.0f;
}

BMM150::~BMM150() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t BMM150::init(const Config& config) {
    esp_err_t ret;

    i2c_port_ = config.i2c_port;
    i2c_addr_ = config.i2c_addr;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Power on (set power control bit)
    ret = writeRegister(REG_POWER_CTRL, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power on");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify chip ID
    uint8_t chip_id = getChipId();
    if (chip_id != CHIP_ID_BMM150) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, CHIP_ID_BMM150);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMM150 detected, chip ID: 0x%02X", chip_id);

    // Read trim registers for compensation
    ret = readTrimRegisters();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read trim registers");
        return ret;
    }

    // Set to normal mode, ODR = 30Hz
    // OP_MODE: [2:1] = 00 (Normal), [5:3] = 110 (30Hz)
    ret = writeRegister(REG_OP_MODE, 0x30);
    if (ret != ESP_OK) return ret;

    // Set repetitions for XY and Z (affects resolution and power)
    // REP_XY = 9 (regular preset), REP_Z = 15 (regular preset)
    ret = writeRegister(REG_REP_XY, 0x09);
    if (ret != ESP_OK) return ret;
    ret = writeRegister(REG_REP_Z, 0x0F);
    if (ret != ESP_OK) return ret;

    initialized_ = true;
    ESP_LOGI(TAG, "BMM150 initialized at address 0x%02X", i2c_addr_);
    return ESP_OK;
}

esp_err_t BMM150::readTrimRegisters() {
    uint8_t data[2];

    readRegister(REG_TRIM_DIG_X1, (uint8_t*)&trim_.dig_x1);
    readRegister(REG_TRIM_DIG_Y1, (uint8_t*)&trim_.dig_y1);
    readRegister(REG_TRIM_DIG_X2, (uint8_t*)&trim_.dig_x2);
    readRegister(REG_TRIM_DIG_Y2, (uint8_t*)&trim_.dig_y2);
    readRegister(REG_TRIM_DIG_XY1, &trim_.dig_xy1);
    readRegister(REG_TRIM_DIG_XY2, (uint8_t*)&trim_.dig_xy2);

    readRegisters(REG_TRIM_DIG_Z1_LSB, data, 2);
    trim_.dig_z1 = (uint16_t)(data[1] << 8) | data[0];

    readRegisters(REG_TRIM_DIG_Z2_LSB, data, 2);
    trim_.dig_z2 = (int16_t)((data[1] << 8) | data[0]);

    readRegisters(REG_TRIM_DIG_Z3_LSB, data, 2);
    trim_.dig_z3 = (int16_t)((data[1] << 8) | data[0]);

    readRegisters(REG_TRIM_DIG_Z4_LSB, data, 2);
    trim_.dig_z4 = (int16_t)((data[1] << 8) | data[0]);

    readRegisters(REG_TRIM_DIG_XYZ1_LSB, data, 2);
    trim_.dig_xyz1 = (uint16_t)((data[1] << 8) | data[0]);

    return ESP_OK;
}

esp_err_t BMM150::read(MagData& data) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read 8 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB, RHALL_LSB, RHALL_MSB
    uint8_t buffer[8];
    esp_err_t ret = readRegisters(REG_DATA_X_LSB, buffer, 8);

    xSemaphoreGive(mutex_);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse raw data (X and Y are 13-bit, Z is 15-bit, RHALL is 14-bit)
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]) >> 3;
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]) >> 3;
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]) >> 1;
    uint16_t rhall = (uint16_t)((buffer[7] << 8) | buffer[6]) >> 2;

    // Apply compensation
    data.x = compensateX(raw_x, rhall);
    data.y = compensateY(raw_y, rhall);
    data.z = compensateZ(raw_z, rhall);

    // Apply hard/soft iron calibration
    float mx = data.x - calibration_.hard_iron[0];
    float my = data.y - calibration_.hard_iron[1];
    float mz = data.z - calibration_.hard_iron[2];

    data.x = calibration_.soft_iron[0] * mx + calibration_.soft_iron[1] * my + calibration_.soft_iron[2] * mz;
    data.y = calibration_.soft_iron[3] * mx + calibration_.soft_iron[4] * my + calibration_.soft_iron[5] * mz;
    data.z = calibration_.soft_iron[6] * mx + calibration_.soft_iron[7] * my + calibration_.soft_iron[8] * mz;

    data.timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

float BMM150::compensateX(int16_t raw_x, uint16_t rhall) {
    if (raw_x == -4096 || rhall == 0 || trim_.dig_xyz1 == 0) {
        return 0.0f;
    }

    float process_comp_x0 = (float)trim_.dig_xyz1 * 16384.0f / (float)rhall;
    float process_comp_x1 = (float)trim_.dig_xy2 * (process_comp_x0 * process_comp_x0 / 268435456.0f - 1.0f);
    float process_comp_x2 = process_comp_x1 + (float)trim_.dig_xy1 * 128.0f;
    float process_comp_x3 = (float)trim_.dig_x2 + 160.0f;
    float process_comp_x4 = (float)raw_x * ((process_comp_x0 - 256.0f) / 8192.0f * process_comp_x3 + process_comp_x2);
    float compensated = process_comp_x4 + (float)trim_.dig_x1 * 8.0f;

    return compensated / 16.0f;  // Convert to uT
}

float BMM150::compensateY(int16_t raw_y, uint16_t rhall) {
    if (raw_y == -4096 || rhall == 0 || trim_.dig_xyz1 == 0) {
        return 0.0f;
    }

    float process_comp_y0 = (float)trim_.dig_xyz1 * 16384.0f / (float)rhall;
    float process_comp_y1 = (float)trim_.dig_xy2 * (process_comp_y0 * process_comp_y0 / 268435456.0f - 1.0f);
    float process_comp_y2 = process_comp_y1 + (float)trim_.dig_xy1 * 128.0f;
    float process_comp_y3 = (float)trim_.dig_y2 + 160.0f;
    float process_comp_y4 = (float)raw_y * ((process_comp_y0 - 256.0f) / 8192.0f * process_comp_y3 + process_comp_y2);
    float compensated = process_comp_y4 + (float)trim_.dig_y1 * 8.0f;

    return compensated / 16.0f;  // Convert to uT
}

float BMM150::compensateZ(int16_t raw_z, uint16_t rhall) {
    if (raw_z == -16384 || rhall == 0 || trim_.dig_z2 == 0 || trim_.dig_z1 == 0) {
        return 0.0f;
    }

    float process_comp_z0 = (float)raw_z - (float)trim_.dig_z4;
    float process_comp_z1 = (float)rhall - (float)trim_.dig_xyz1;
    float process_comp_z2 = (float)trim_.dig_z3 * process_comp_z1;
    float process_comp_z3 = (float)trim_.dig_z1 * (float)rhall / 32768.0f;
    float process_comp_z4 = (float)trim_.dig_z2 + process_comp_z3;
    float compensated = (process_comp_z0 - process_comp_z2) / process_comp_z4;

    return compensated;  // Already in uT
}

void BMM150::setCalibration(const CalibrationData& cal) {
    calibration_ = cal;
}

BMM150::CalibrationData BMM150::getCalibration() const {
    return calibration_;
}

bool BMM150::isDataReady() {
    uint8_t status;
    if (readRegister(REG_STATUS, &status) == ESP_OK) {
        return (status & 0x01) != 0;
    }
    return false;
}

uint8_t BMM150::getChipId() {
    uint8_t id = 0;
    readRegister(REG_CHIP_ID, &id);
    return id;
}

esp_err_t BMM150::softReset() {
    return writeRegister(REG_POWER_CTRL, 0x82);  // Soft reset + power on
}

esp_err_t BMM150::writeRegister(uint8_t reg, uint8_t value) {
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

esp_err_t BMM150::readRegister(uint8_t reg, uint8_t* value) {
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

esp_err_t BMM150::readRegisters(uint8_t reg, uint8_t* buffer, size_t len) {
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
