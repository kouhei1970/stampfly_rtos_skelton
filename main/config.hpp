/**
 * @file config.hpp
 * @brief ハードウェア設定とタスク設定
 *
 * GPIO定義、タスク優先度、スタックサイズなどの定数を集約
 */

#pragma once

#include "freertos/FreeRTOS.h"

namespace config {

// =============================================================================
// GPIO Definitions
// =============================================================================

// SPI Bus
inline constexpr int GPIO_SPI_MOSI = 14;
inline constexpr int GPIO_SPI_MISO = 43;
inline constexpr int GPIO_SPI_SCK = 44;
inline constexpr int GPIO_IMU_CS = 46;
inline constexpr int GPIO_FLOW_CS = 12;

// I2C Bus
inline constexpr int GPIO_I2C_SDA = 3;
inline constexpr int GPIO_I2C_SCL = 4;

// ToF XSHUT
inline constexpr int GPIO_TOF_XSHUT_BOTTOM = 7;
inline constexpr int GPIO_TOF_XSHUT_FRONT = 9;

// Motors (LEDC PWM)
inline constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
inline constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
inline constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
inline constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// Peripherals
inline constexpr int GPIO_LED = 39;
inline constexpr int GPIO_BUZZER = 40;
inline constexpr int GPIO_BUTTON = 0;

// =============================================================================
// Task Priorities
// =============================================================================

inline constexpr UBaseType_t PRIORITY_IMU_TASK = 24;
inline constexpr UBaseType_t PRIORITY_CONTROL_TASK = 23;
inline constexpr UBaseType_t PRIORITY_OPTFLOW_TASK = 20;
inline constexpr UBaseType_t PRIORITY_MAG_TASK = 18;
inline constexpr UBaseType_t PRIORITY_BARO_TASK = 16;
inline constexpr UBaseType_t PRIORITY_COMM_TASK = 15;
inline constexpr UBaseType_t PRIORITY_TOF_TASK = 14;
inline constexpr UBaseType_t PRIORITY_TELEMETRY_TASK = 13;
inline constexpr UBaseType_t PRIORITY_POWER_TASK = 12;
inline constexpr UBaseType_t PRIORITY_BUTTON_TASK = 10;
inline constexpr UBaseType_t PRIORITY_LED_TASK = 8;
inline constexpr UBaseType_t PRIORITY_CLI_TASK = 5;

// =============================================================================
// Task Stack Sizes
// =============================================================================

inline constexpr uint32_t STACK_SIZE_IMU = 16384;      // ESKF行列演算用
inline constexpr uint32_t STACK_SIZE_CONTROL = 8192;
inline constexpr uint32_t STACK_SIZE_OPTFLOW = 8192;
inline constexpr uint32_t STACK_SIZE_MAG = 8192;
inline constexpr uint32_t STACK_SIZE_BARO = 8192;
inline constexpr uint32_t STACK_SIZE_TOF = 8192;
inline constexpr uint32_t STACK_SIZE_POWER = 4096;
inline constexpr uint32_t STACK_SIZE_LED = 4096;
inline constexpr uint32_t STACK_SIZE_BUTTON = 4096;
inline constexpr uint32_t STACK_SIZE_COMM = 4096;
inline constexpr uint32_t STACK_SIZE_CLI = 4096;
inline constexpr uint32_t STACK_SIZE_TELEMETRY = 4096;

// =============================================================================
// Timing Constants
// =============================================================================

inline constexpr float IMU_DT = 0.0025f;          // 400Hz
inline constexpr float OPTFLOW_DT = 0.01f;        // 100Hz
inline constexpr float MAG_DT = 0.01f;            // 100Hz
inline constexpr float BARO_DT = 0.02f;           // 50Hz
inline constexpr float TOF_DT = 0.033f;           // ~30Hz

// =============================================================================
// Sensor Thresholds
// =============================================================================

// Optical Flow
inline constexpr uint8_t FLOW_SQUAL_MIN = 0x19;   // 最小品質閾値
inline constexpr float FLOW_DISTANCE_MIN = 0.02f; // [m]
inline constexpr float FLOW_DISTANCE_MAX = 4.0f;  // [m]

// ToF
inline constexpr float TOF_DISTANCE_MIN = 0.01f;  // [m]
inline constexpr float TOF_DISTANCE_MAX = 4.0f;   // [m]

// ESKF Divergence Detection
inline constexpr float ESKF_MAX_POSITION = 100.0f;  // [m]
inline constexpr float ESKF_MAX_VELOCITY = 50.0f;   // [m/s]

} // namespace config
