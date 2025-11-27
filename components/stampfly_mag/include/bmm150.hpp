/**
 * @file bmm150.hpp
 * @brief BMM150 Magnetometer Driver for StampFly
 *
 * I2C interface driver for Bosch BMM150 3-axis magnetometer
 */
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

class BMM150 {
public:
    struct Config {
        i2c_port_t i2c_port;
        uint8_t i2c_addr;
    };

    struct MagData {
        float x, y, z;       // [uT] micro Tesla
        int64_t timestamp_us;
    };

    struct CalibrationData {
        float hard_iron[3];  // Hard iron offset [uT]
        float soft_iron[9];  // Soft iron matrix (3x3)
    };

    BMM150();
    ~BMM150();

    /**
     * @brief Initialize the sensor
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read magnetometer data
     * @param data Output mag data structure
     * @return ESP_OK on success
     */
    esp_err_t read(MagData& data);

    /**
     * @brief Set calibration data
     * @param cal Calibration data
     */
    void setCalibration(const CalibrationData& cal);

    /**
     * @brief Get current calibration
     * @return Calibration data
     */
    CalibrationData getCalibration() const;

    /**
     * @brief Check if data is ready
     * @return true if new data available
     */
    bool isDataReady();

    /**
     * @brief Get chip ID
     * @return Chip ID (should be 0x32 for BMM150)
     */
    uint8_t getChipId();

    /**
     * @brief Perform soft reset
     * @return ESP_OK on success
     */
    esp_err_t softReset();

private:
    esp_err_t writeRegister(uint8_t reg, uint8_t value);
    esp_err_t readRegister(uint8_t reg, uint8_t* value);
    esp_err_t readRegisters(uint8_t reg, uint8_t* buffer, size_t len);

    esp_err_t readTrimRegisters();

    i2c_port_t i2c_port_;
    uint8_t i2c_addr_;
    SemaphoreHandle_t mutex_;
    bool initialized_;
    CalibrationData calibration_;

    // Trim data for compensation
    struct TrimData {
        int8_t dig_x1;
        int8_t dig_y1;
        int8_t dig_x2;
        int8_t dig_y2;
        uint16_t dig_z1;
        int16_t dig_z2;
        int16_t dig_z3;
        int16_t dig_z4;
        uint8_t dig_xy1;
        int8_t dig_xy2;
        uint16_t dig_xyz1;
    } trim_;

    // BMM150 Registers
    static constexpr uint8_t REG_CHIP_ID = 0x40;
    static constexpr uint8_t REG_DATA_X_LSB = 0x42;
    static constexpr uint8_t REG_DATA_X_MSB = 0x43;
    static constexpr uint8_t REG_DATA_Y_LSB = 0x44;
    static constexpr uint8_t REG_DATA_Y_MSB = 0x45;
    static constexpr uint8_t REG_DATA_Z_LSB = 0x46;
    static constexpr uint8_t REG_DATA_Z_MSB = 0x47;
    static constexpr uint8_t REG_RHALL_LSB = 0x48;
    static constexpr uint8_t REG_RHALL_MSB = 0x49;
    static constexpr uint8_t REG_STATUS = 0x4A;
    static constexpr uint8_t REG_POWER_CTRL = 0x4B;
    static constexpr uint8_t REG_OP_MODE = 0x4C;
    static constexpr uint8_t REG_INT_CTRL = 0x4D;
    static constexpr uint8_t REG_AXES_ENABLE = 0x4E;
    static constexpr uint8_t REG_REP_XY = 0x51;
    static constexpr uint8_t REG_REP_Z = 0x52;

    // Trim register addresses
    static constexpr uint8_t REG_TRIM_DIG_X1 = 0x5D;
    static constexpr uint8_t REG_TRIM_DIG_Y1 = 0x5E;
    static constexpr uint8_t REG_TRIM_DIG_Z4_LSB = 0x62;
    static constexpr uint8_t REG_TRIM_DIG_X2 = 0x64;
    static constexpr uint8_t REG_TRIM_DIG_Y2 = 0x65;
    static constexpr uint8_t REG_TRIM_DIG_Z2_LSB = 0x68;
    static constexpr uint8_t REG_TRIM_DIG_Z1_LSB = 0x6A;
    static constexpr uint8_t REG_TRIM_DIG_XYZ1_LSB = 0x6C;
    static constexpr uint8_t REG_TRIM_DIG_Z3_LSB = 0x6E;
    static constexpr uint8_t REG_TRIM_DIG_XY2 = 0x70;
    static constexpr uint8_t REG_TRIM_DIG_XY1 = 0x71;

    // Expected values
    static constexpr uint8_t CHIP_ID_BMM150 = 0x32;

    // Compensation calculation
    float compensateX(int16_t raw_x, uint16_t rhall);
    float compensateY(int16_t raw_y, uint16_t rhall);
    float compensateZ(int16_t raw_z, uint16_t rhall);
};

}  // namespace stampfly
