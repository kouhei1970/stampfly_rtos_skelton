/**
 * @file bmp280.hpp
 * @brief BMP280 Barometer Driver for StampFly
 *
 * I2C interface driver for Bosch BMP280 pressure/temperature sensor
 */
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

class BMP280 {
public:
    struct Config {
        i2c_port_t i2c_port;
        uint8_t i2c_addr;
    };

    struct BaroData {
        float pressure;      // [hPa]
        float temperature;   // [°C]
        float altitude;      // [m] relative to sea level
        int64_t timestamp_us;
    };

    BMP280();
    ~BMP280();

    /**
     * @brief Initialize the sensor
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read pressure and temperature
     * @param data Output data structure
     * @return ESP_OK on success
     */
    esp_err_t read(BaroData& data);

    /**
     * @brief Set sea level pressure for altitude calculation
     * @param pressure_hpa Sea level pressure in hPa
     */
    void setSeaLevelPressure(float pressure_hpa);

    /**
     * @brief Set reference altitude (for relative altitude)
     * @param altitude_m Reference altitude in meters
     */
    void setReferenceAltitude(float altitude_m);

    /**
     * @brief Check if data is ready
     * @return true if new data available
     */
    bool isDataReady();

    /**
     * @brief Get chip ID
     * @return Chip ID (0x58 for BMP280, 0x60 for BME280)
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

    esp_err_t readCalibration();
    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);
    float calculateAltitude(float pressure);

    i2c_port_t i2c_port_;
    uint8_t i2c_addr_;
    SemaphoreHandle_t mutex_;
    bool initialized_;
    float sea_level_pressure_;
    float reference_altitude_;
    int32_t t_fine_;  // For compensation calculation

    // Calibration data
    struct CalibData {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
    } calib_;

    // BMP280 Registers
    static constexpr uint8_t REG_CALIB_00 = 0x88;
    static constexpr uint8_t REG_CHIP_ID = 0xD0;
    static constexpr uint8_t REG_SOFT_RESET = 0xE0;
    static constexpr uint8_t REG_STATUS = 0xF3;
    static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
    static constexpr uint8_t REG_CONFIG = 0xF5;
    static constexpr uint8_t REG_PRESS_MSB = 0xF7;
    static constexpr uint8_t REG_PRESS_LSB = 0xF8;
    static constexpr uint8_t REG_PRESS_XLSB = 0xF9;
    static constexpr uint8_t REG_TEMP_MSB = 0xFA;
    static constexpr uint8_t REG_TEMP_LSB = 0xFB;
    static constexpr uint8_t REG_TEMP_XLSB = 0xFC;

    // Expected values
    static constexpr uint8_t CHIP_ID_BMP280 = 0x58;
    static constexpr uint8_t CHIP_ID_BME280 = 0x60;
    static constexpr uint8_t CMD_SOFT_RESET = 0xB6;
};

}  // namespace stampfly
