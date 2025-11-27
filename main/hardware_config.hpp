/**
 * @file hardware_config.hpp
 * @brief StampFly Hardware Configuration
 *
 * GPIO assignments and hardware constants for StampFly drone
 */

#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

namespace stampfly {
namespace hw {

// =============================================================================
// SPI Bus Configuration
// =============================================================================
namespace spi {
    constexpr spi_host_device_t HOST = SPI2_HOST;
    constexpr gpio_num_t MOSI = GPIO_NUM_14;
    constexpr gpio_num_t MISO = GPIO_NUM_43;
    constexpr gpio_num_t SCLK = GPIO_NUM_44;
    constexpr int DMA_CHANNEL = SPI_DMA_CH_AUTO;
    constexpr int MAX_TRANSFER_SIZE = 4096;
}

// =============================================================================
// I2C Bus Configuration
// =============================================================================
namespace i2c {
    constexpr i2c_port_t PORT = I2C_NUM_0;
    constexpr gpio_num_t SDA = GPIO_NUM_3;
    constexpr gpio_num_t SCL = GPIO_NUM_4;
    constexpr uint32_t FREQ_HZ = 400000;  // 400kHz
}

// Grove I2C (optional expansion)
namespace grove_i2c {
    constexpr gpio_num_t SDA = GPIO_NUM_13;
    constexpr gpio_num_t SCL = GPIO_NUM_15;
}

// Grove UART (optional expansion)
namespace grove_uart {
    constexpr gpio_num_t RX = GPIO_NUM_1;
    constexpr gpio_num_t TX = GPIO_NUM_2;
}

// =============================================================================
// IMU (BMI270) Configuration
// =============================================================================
namespace imu {
    constexpr gpio_num_t CS = GPIO_NUM_46;
    constexpr gpio_num_t INT1 = GPIO_NUM_11;
    constexpr int CLOCK_SPEED_HZ = 10000000;  // 10MHz
    constexpr uint16_t SAMPLE_RATE_HZ = 400;
    constexpr uint16_t INTERNAL_SAMPLE_RATE_HZ = 1600;
}

// =============================================================================
// Optical Flow (PMW3901) Configuration
// =============================================================================
namespace optflow {
    constexpr gpio_num_t CS = GPIO_NUM_12;
    constexpr int CLOCK_SPEED_HZ = 2000000;  // 2MHz
    constexpr uint16_t SAMPLE_RATE_HZ = 100;
}

// =============================================================================
// ToF Sensors (VL53L3CX) Configuration
// =============================================================================
namespace tof {
    // Front sensor
    namespace front {
        constexpr gpio_num_t XSHUT = GPIO_NUM_9;
        constexpr gpio_num_t INT = GPIO_NUM_8;
        constexpr uint8_t I2C_ADDR = 0x29;
    }
    // Bottom sensor
    namespace bottom {
        constexpr gpio_num_t XSHUT = GPIO_NUM_7;
        constexpr gpio_num_t INT = GPIO_NUM_6;
        constexpr uint8_t I2C_ADDR = 0x30;
    }
    constexpr uint16_t SAMPLE_RATE_HZ = 30;
}

// =============================================================================
// Magnetometer (BMM150) Configuration
// =============================================================================
namespace mag {
    constexpr uint8_t I2C_ADDR = 0x10;
    constexpr uint16_t SAMPLE_RATE_HZ = 100;
}

// =============================================================================
// Barometer (BMP280) Configuration
// =============================================================================
namespace baro {
    constexpr uint8_t I2C_ADDR = 0x76;
    constexpr uint16_t SAMPLE_RATE_HZ = 50;
}

// =============================================================================
// Power Monitor (INA3221) Configuration
// =============================================================================
namespace power {
    constexpr uint8_t I2C_ADDR = 0x40;
    constexpr uint8_t BATTERY_CHANNEL = 2;
    constexpr float SHUNT_RESISTANCE_MOHM = 10.0f;
    constexpr float BATTERY_LOW_VOLTAGE = 3.4f;
    constexpr float BATTERY_FULL_VOLTAGE = 4.2f;
    constexpr float BATTERY_MAX_VOLTAGE = 4.35f;  // LiHV
    constexpr uint16_t SAMPLE_RATE_HZ = 10;
}

// =============================================================================
// Motor Configuration (PWM)
// =============================================================================
namespace motor {
    constexpr uint8_t NUM_MOTORS = 4;

    // Motor GPIO pins (M1-M4, clockwise from front-right)
    constexpr gpio_num_t PINS[NUM_MOTORS] = {
        GPIO_NUM_5,   // M1: Front Right (CCW)
        GPIO_NUM_45,  // M2: Rear Right (CW)
        GPIO_NUM_41,  // M3: Rear Left (CCW)
        GPIO_NUM_42   // M4: Front Left (CW)
    };

    // Motor rotation directions
    enum class Direction : uint8_t {
        CW = 0,   // Clockwise
        CCW = 1   // Counter-clockwise
    };

    constexpr Direction DIRECTIONS[NUM_MOTORS] = {
        Direction::CCW,  // M1: Front Right
        Direction::CW,   // M2: Rear Right
        Direction::CCW,  // M3: Rear Left
        Direction::CW    // M4: Front Left
    };

    // PWM configuration
    constexpr uint32_t PWM_FREQ_HZ = 50000;  // 50kHz
    constexpr uint8_t PWM_RESOLUTION_BITS = 11;  // 0-2047
    constexpr uint16_t MAX_THROTTLE = 2000;
    constexpr uint16_t MIN_THROTTLE = 0;
    constexpr uint16_t IDLE_THROTTLE = 100;

    // LEDC channels for motors
    constexpr ledc_channel_t LEDC_CHANNELS[NUM_MOTORS] = {
        LEDC_CHANNEL_0,
        LEDC_CHANNEL_1,
        LEDC_CHANNEL_2,
        LEDC_CHANNEL_3
    };
    constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
}

// =============================================================================
// Buzzer Configuration
// =============================================================================
namespace buzzer {
    constexpr gpio_num_t PIN = GPIO_NUM_40;
    constexpr ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_5;
    constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_2;

    // Note frequencies
    enum Note : uint16_t {
        NOTE_D1 = 294,
        NOTE_D2 = 330,
        NOTE_D3 = 350,
        NOTE_D4 = 393,
        NOTE_D5 = 441,
        NOTE_D6 = 495,
        NOTE_D7 = 556
    };
}

// =============================================================================
// RGB LED Configuration (WS2812)
// =============================================================================
namespace led {
    constexpr gpio_num_t PIN_ONBOARD = GPIO_NUM_39;  // 2 LEDs
    constexpr gpio_num_t PIN_ESP = GPIO_NUM_21;      // 1 LED
    constexpr uint8_t NUM_ONBOARD = 2;
    constexpr uint8_t NUM_ESP = 1;
    constexpr uint8_t DEFAULT_BRIGHTNESS = 15;

    // Color definitions (RGB)
    enum Color : uint32_t {
        WHITE = 0xFFFFFF,
        RED = 0xFF0000,
        GREEN = 0x00FF00,
        BLUE = 0x0000FF,
        YELLOW = 0xFFFF00,
        PURPLE = 0xFF00FF,
        CYAN = 0x00FFFF,
        ORANGE = 0xFF9933,
        LOW_BATTERY = 0x18EBF9  // Cyan-ish
    };
}

// =============================================================================
// Button Configuration
// =============================================================================
namespace button {
    constexpr gpio_num_t PIN = GPIO_NUM_0;
    constexpr bool ACTIVE_LOW = true;
    constexpr uint32_t DEBOUNCE_MS = 50;
    constexpr uint32_t LONG_PRESS_MS = 3000;  // 3 seconds
}

// =============================================================================
// Communication Configuration
// =============================================================================
namespace comm {
    constexpr uint8_t WIFI_CHANNEL = 6;
    constexpr uint32_t CONTROL_TIMEOUT_MS = 500;
    constexpr uint16_t TELEMETRY_RATE_HZ = 50;

    // Pairing magic number
    constexpr uint8_t PAIRING_MAGIC[4] = {0xAA, 0x55, 0x16, 0x88};
}

// =============================================================================
// Task Configuration
// =============================================================================
namespace task {
    // Stack sizes
    constexpr uint32_t STACK_IMU = 4096;
    constexpr uint32_t STACK_MAG = 2048;
    constexpr uint32_t STACK_BARO = 2048;
    constexpr uint32_t STACK_TOF = 4096;
    constexpr uint32_t STACK_FLOW = 4096;
    constexpr uint32_t STACK_POWER = 2048;
    constexpr uint32_t STACK_LED = 2048;
    constexpr uint32_t STACK_BUTTON = 2048;
    constexpr uint32_t STACK_COMM = 4096;
    constexpr uint32_t STACK_MAIN = 8192;
    constexpr uint32_t STACK_CLI = 4096;

    // Task priorities (higher = more priority)
    constexpr uint8_t PRIORITY_IMU = 24;
    constexpr uint8_t PRIORITY_MAIN = 23;
    constexpr uint8_t PRIORITY_COMM = 22;
    constexpr uint8_t PRIORITY_FLOW = 20;
    constexpr uint8_t PRIORITY_MAG = 18;
    constexpr uint8_t PRIORITY_BARO = 16;
    constexpr uint8_t PRIORITY_TOF = 14;
    constexpr uint8_t PRIORITY_POWER = 12;
    constexpr uint8_t PRIORITY_BUTTON = 10;
    constexpr uint8_t PRIORITY_LED = 8;
    constexpr uint8_t PRIORITY_CLI = 5;
}

// =============================================================================
// NVS Configuration
// =============================================================================
namespace nvs {
    constexpr const char* NAMESPACE = "stampfly";
    constexpr const char* KEY_CTRL_MAC = "ctrl_mac";
    constexpr const char* KEY_GYRO_BIAS = "gyro_bias";
    constexpr const char* KEY_ACCEL_BIAS = "accel_bias";
    constexpr const char* KEY_MAG_HARD_IRON = "mag_hard";
    constexpr const char* KEY_MAG_SOFT_IRON = "mag_soft";
    constexpr const char* KEY_BARO_OFFSET = "baro_off";
}

}  // namespace hw
}  // namespace stampfly
