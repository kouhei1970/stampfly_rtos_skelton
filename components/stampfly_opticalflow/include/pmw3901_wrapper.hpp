/**
 * @file pmw3901_wrapper.hpp
 * @brief PMW3901 Optical Flow Sensor C++ Wrapper
 *
 * Burst read support, SQUAL quality check
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

struct MotionData {
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal;      // Surface quality (0-255)
    uint8_t shutter_upper;
    uint8_t shutter_lower;
    uint32_t timestamp_us;
};

class PMW3901Wrapper {
public:
    struct Config {
        int spi_host;
        int cs_gpio;
        int spi_freq_hz;
    };

    PMW3901Wrapper() = default;
    ~PMW3901Wrapper() = default;

    /**
     * @brief Initialize PMW3901
     * @param config SPI configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read motion data
     * @param data Motion data output
     * @return ESP_OK on success
     */
    esp_err_t readMotion(MotionData& data);

    /**
     * @brief Check if motion data is valid (SQUAL >= 30)
     * @param data Motion data to check
     * @return true if valid
     */
    static bool isMotionValid(const MotionData& data) {
        return data.squal >= 30;
    }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
};

}  // namespace stampfly
