/**
 * @file pmw3901_wrapper.hpp
 * @brief PMW3901 Optical Flow Sensor Wrapper (Placeholder)
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class PMW3901 {
public:
    struct Config {
        int spi_host;
        int cs_pin;
    };

    struct FlowData {
        int16_t delta_x;
        int16_t delta_y;
        uint8_t squal;
    };

    esp_err_t init(const Config& config);
    esp_err_t read(FlowData& data);
};

}  // namespace stampfly
