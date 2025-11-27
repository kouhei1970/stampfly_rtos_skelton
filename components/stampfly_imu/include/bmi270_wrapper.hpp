/**
 * @file bmi270_wrapper.hpp
 * @brief BMI270 IMU Driver for StampFly
 *
 * SPI interface driver for Bosch BMI270 6-axis IMU
 */
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

class BMI270 {
public:
    struct Config {
        spi_host_device_t spi_host;
        gpio_num_t cs_pin;
        gpio_num_t int1_pin;
        int clock_speed_hz;
        uint16_t sample_rate_hz;  // Output data rate
    };

    struct ImuData {
        float accel_x, accel_y, accel_z;  // [m/s^2]
        float gyro_x, gyro_y, gyro_z;     // [rad/s]
        int64_t timestamp_us;              // Timestamp in microseconds
    };

    struct CalibrationData {
        float gyro_bias[3];   // [rad/s]
        float accel_bias[3];  // [m/s^2]
    };

    BMI270();
    ~BMI270();

    /**
     * @brief Initialize the BMI270 sensor
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read sensor data
     * @param data Output data structure
     * @return ESP_OK on success
     */
    esp_err_t read(ImuData& data);

    /**
     * @brief Perform gyroscope calibration (keep sensor still)
     * @param num_samples Number of samples to average
     * @return ESP_OK on success
     */
    esp_err_t calibrateGyro(uint16_t num_samples = 1000);

    /**
     * @brief Set calibration data
     * @param cal Calibration data
     */
    void setCalibration(const CalibrationData& cal);

    /**
     * @brief Get current calibration data
     * @return Calibration data
     */
    CalibrationData getCalibration() const;

    /**
     * @brief Check if data ready interrupt is asserted
     * @return true if new data available
     */
    bool isDataReady();

    /**
     * @brief Get chip ID
     * @return Chip ID (should be 0x24 for BMI270)
     */
    uint8_t getChipId();

    /**
     * @brief Perform soft reset
     * @return ESP_OK on success
     */
    esp_err_t softReset();

private:
    // SPI communication
    esp_err_t writeRegister(uint8_t reg, uint8_t value);
    esp_err_t readRegister(uint8_t reg, uint8_t* value);
    esp_err_t readRegisters(uint8_t reg, uint8_t* buffer, size_t len);
    esp_err_t burstRead(uint8_t reg, uint8_t* buffer, size_t len);

    // Configuration
    esp_err_t uploadConfigFile();
    esp_err_t configureSensor();

    spi_device_handle_t spi_handle_;
    gpio_num_t int1_pin_;
    Config config_;
    CalibrationData calibration_;
    SemaphoreHandle_t mutex_;
    bool initialized_;

    // Conversion factors
    static constexpr float ACCEL_SCALE_16G = 16.0f * 9.80665f / 32768.0f;  // to m/s^2
    static constexpr float GYRO_SCALE_2000DPS = 2000.0f * (3.14159265f / 180.0f) / 32768.0f;  // to rad/s

    // BMI270 Registers
    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_ERR_REG = 0x02;
    static constexpr uint8_t REG_STATUS = 0x03;
    static constexpr uint8_t REG_DATA_0 = 0x04;  // Aux data
    static constexpr uint8_t REG_DATA_8 = 0x0C;  // Accel X LSB
    static constexpr uint8_t REG_DATA_14 = 0x12; // Gyro X LSB
    static constexpr uint8_t REG_SENSORTIME_0 = 0x18;
    static constexpr uint8_t REG_INT_STATUS_1 = 0x1D;
    static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
    static constexpr uint8_t REG_ACC_CONF = 0x40;
    static constexpr uint8_t REG_ACC_RANGE = 0x41;
    static constexpr uint8_t REG_GYR_CONF = 0x42;
    static constexpr uint8_t REG_GYR_RANGE = 0x43;
    static constexpr uint8_t REG_INT1_IO_CTRL = 0x53;
    static constexpr uint8_t REG_INT1_MAP_FEAT = 0x56;
    static constexpr uint8_t REG_INT_MAP_DATA = 0x58;
    static constexpr uint8_t REG_INIT_CTRL = 0x59;
    static constexpr uint8_t REG_INIT_DATA = 0x5E;
    static constexpr uint8_t REG_PWR_CONF = 0x7C;
    static constexpr uint8_t REG_PWR_CTRL = 0x7D;
    static constexpr uint8_t REG_CMD = 0x7E;

    // BMI270 Values
    static constexpr uint8_t CHIP_ID_BMI270 = 0x24;
    static constexpr uint8_t CMD_SOFT_RESET = 0xB6;
};

}  // namespace stampfly
