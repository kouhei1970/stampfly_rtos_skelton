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

// =============================================================================
// ESKF (Error-State Kalman Filter) Configuration
// =============================================================================
//
// センサーフュージョン（状態推定）の全設定
// 学習者へ: ここで全てのESKFパラメータを確認・変更できます
//

namespace eskf {

// -----------------------------------------------------------------------------
// センサー有効/無効スイッチ
// デフォルト: 全センサーON。デバッグ時に個別無効化可能
// -----------------------------------------------------------------------------
inline constexpr bool USE_OPTICAL_FLOW = true;     // オプティカルフロー（水平速度推定）
inline constexpr bool USE_BAROMETER = true;        // 気圧センサー（高度推定）
inline constexpr bool USE_TOF = true;              // ToFセンサー（高度推定）
inline constexpr bool USE_MAGNETOMETER = true;     // 地磁気センサー（ヨー推定）

// -----------------------------------------------------------------------------
// プロセスノイズ (Q行列)
// 値が大きい = センサーを信頼、値が小さい = モデルを信頼
// -----------------------------------------------------------------------------
inline constexpr float GYRO_NOISE = 0.009655f;         // ジャイロノイズ [rad/s/√Hz]
inline constexpr float ACCEL_NOISE = 0.062885f;        // 加速度ノイズ [m/s²/√Hz]
inline constexpr float GYRO_BIAS_NOISE = 0.000013f;    // ジャイロバイアスランダムウォーク
inline constexpr float ACCEL_BIAS_NOISE = 0.0001f;     // 加速度バイアスランダムウォーク

// -----------------------------------------------------------------------------
// 観測ノイズ (R行列)
// 値が大きい = 観測を信頼しない、値が小さい = 観測を信頼
// -----------------------------------------------------------------------------
inline constexpr float BARO_NOISE = 0.1f;              // 気圧高度ノイズ [m]
inline constexpr float TOF_NOISE = 0.002540f;          // ToFノイズ [m]
inline constexpr float MAG_NOISE = 2.0f;               // 地磁気ノイズ [uT]
inline constexpr float FLOW_NOISE = 0.005232f;         // オプティカルフローノイズ [m/s]
inline constexpr float ACCEL_ATT_NOISE = 0.02f;        // 加速度計姿勢補正ノイズ [m/s²]

// -----------------------------------------------------------------------------
// 初期共分散 (P行列の初期値)
// -----------------------------------------------------------------------------
inline constexpr float INIT_POS_STD = 1.0f;            // 位置 [m]
inline constexpr float INIT_VEL_STD = 0.5f;            // 速度 [m/s]
inline constexpr float INIT_ATT_STD = 0.1f;            // 姿勢 [rad]
inline constexpr float INIT_GYRO_BIAS_STD = 0.01f;     // ジャイロバイアス [rad/s]
inline constexpr float INIT_ACCEL_BIAS_STD = 0.1f;     // 加速度バイアス [m/s²]

// -----------------------------------------------------------------------------
// 物理定数
// -----------------------------------------------------------------------------
inline constexpr float GRAVITY = 9.81f;                // 重力加速度 [m/s²]

// 地磁気参照ベクトル (NED座標系) - 日本近辺の概算値
inline constexpr float MAG_REF_X = 20.0f;              // 北成分 [uT]
inline constexpr float MAG_REF_Y = 0.0f;               // 東成分 [uT]
inline constexpr float MAG_REF_Z = 40.0f;              // 下成分 [uT]

// -----------------------------------------------------------------------------
// 閾値設定
// -----------------------------------------------------------------------------
inline constexpr float MAHALANOBIS_THRESHOLD = 15.0f;  // アウトライア棄却閾値
inline constexpr float TOF_TILT_THRESHOLD = 0.70f;     // ToF傾き閾値 [rad] (~40度)
inline constexpr float ACCEL_MOTION_THRESHOLD = 1.0f;  // 加速度モーション閾値 [m/s²]

// 発散検出閾値
inline constexpr float MAX_POSITION = 100.0f;          // [m]
inline constexpr float MAX_VELOCITY = 50.0f;           // [m/s]

// -----------------------------------------------------------------------------
// オプティカルフロー設定
// -----------------------------------------------------------------------------
inline constexpr float FLOW_MIN_HEIGHT = 0.02f;        // 最小高度 [m]
inline constexpr float FLOW_MAX_HEIGHT = 4.0f;         // 最大高度 [m]
inline constexpr float FLOW_TILT_COS_THRESHOLD = 0.866f; // 傾き閾値 cos(30°)

// PMW3901キャリブレーション
inline constexpr float FLOW_RAD_PER_PIXEL = 0.00222f;  // [rad/pixel]

// カメラ→機体座標変換行列 (2x2)
inline constexpr float FLOW_CAM_TO_BODY_XX = 0.943f;   // X軸スケール
inline constexpr float FLOW_CAM_TO_BODY_XY = 0.0f;
inline constexpr float FLOW_CAM_TO_BODY_YX = 0.0f;
inline constexpr float FLOW_CAM_TO_BODY_YY = 1.015f;   // Y軸スケール

// ジャイロ回転補償スケール
inline constexpr float FLOW_GYRO_SCALE = 1.0f;

// フローオフセット [counts/sample] (キャリブレーション後に設定)
inline constexpr float FLOW_OFFSET_X = 0.0f;
inline constexpr float FLOW_OFFSET_Y = 0.0f;

// -----------------------------------------------------------------------------
// 姿勢補正モード
// 0: 加速度絶対値フィルタのみ
// 1: 適応的R (水平加速度でRをスケーリング)
// 2: 角速度フィルタ (高回転時にRを増加)
// 3: 高回転時バイアス保護
// -----------------------------------------------------------------------------
inline constexpr int ATT_UPDATE_MODE = 0;
inline constexpr float K_ADAPTIVE = 0.0f;
inline constexpr float GYRO_ATT_THRESHOLD = 0.5f;      // [rad/s]

} // namespace eskf

} // namespace config
