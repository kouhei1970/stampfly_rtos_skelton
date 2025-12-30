/**
 * @file main.cpp
 * @brief StampFly RTOS Skeleton - Main Entry Point
 *
 * This is the main entry point for the StampFly flight controller skeleton.
 * It initializes all subsystems and starts the FreeRTOS tasks.
 */

#include <stdio.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

// Sensor drivers
#include "bmi270_wrapper.hpp"
#include "bmm150.hpp"
#include "mag_calibration.hpp"
#include "bmp280.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "power_monitor.hpp"

// Actuators and peripherals
#include "motor_driver.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include "button.hpp"

// State management and estimation
#include "stampfly_state.hpp"
#include "system_manager.hpp"
#include "eskf.hpp"
#include "filter.hpp"

// Communication
#include "controller_comm.hpp"

// CLI
#include "cli.hpp"

// Logger
#include "logger.hpp"

// Telemetry
#include "telemetry.hpp"

static const char* TAG = "main";

// =============================================================================
// GPIO Definitions (from implementation_plan.md)
// =============================================================================

// SPI Bus
static constexpr int GPIO_SPI_MOSI = 14;
static constexpr int GPIO_SPI_MISO = 43;
static constexpr int GPIO_SPI_SCK = 44;
static constexpr int GPIO_IMU_CS = 46;
static constexpr int GPIO_FLOW_CS = 12;

// I2C Bus
static constexpr int GPIO_I2C_SDA = 3;
static constexpr int GPIO_I2C_SCL = 4;

// ToF XSHUT
static constexpr int GPIO_TOF_XSHUT_BOTTOM = 7;
static constexpr int GPIO_TOF_XSHUT_FRONT = 9;

// Motors (LEDC PWM)
static constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
static constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
static constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
static constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// Peripherals
static constexpr int GPIO_LED = 39;
static constexpr int GPIO_BUZZER = 40;
static constexpr int GPIO_BUTTON = 0;

// =============================================================================
// Task Priorities (from implementation_plan.md)
// =============================================================================

static constexpr UBaseType_t PRIORITY_IMU_TASK = 24;
static constexpr UBaseType_t PRIORITY_CONTROL_TASK = 23;  // Flight control (after IMU)
static constexpr UBaseType_t PRIORITY_OPTFLOW_TASK = 20;
static constexpr UBaseType_t PRIORITY_MAG_TASK = 18;
static constexpr UBaseType_t PRIORITY_BARO_TASK = 16;
static constexpr UBaseType_t PRIORITY_COMM_TASK = 15;
static constexpr UBaseType_t PRIORITY_TOF_TASK = 14;
static constexpr UBaseType_t PRIORITY_POWER_TASK = 12;
static constexpr UBaseType_t PRIORITY_BUTTON_TASK = 10;
static constexpr UBaseType_t PRIORITY_LED_TASK = 8;
static constexpr UBaseType_t PRIORITY_CLI_TASK = 5;
static constexpr UBaseType_t PRIORITY_TELEMETRY_TASK = 13;  // Core 0, below ToF

// =============================================================================
// Task Stack Sizes
// =============================================================================

static constexpr uint32_t STACK_SIZE_IMU = 16384;  // Increased for ESKF matrix operations
static constexpr uint32_t STACK_SIZE_CONTROL = 8192;  // Flight control task
static constexpr uint32_t STACK_SIZE_OPTFLOW = 8192;
static constexpr uint32_t STACK_SIZE_MAG = 8192;
static constexpr uint32_t STACK_SIZE_BARO = 8192;
static constexpr uint32_t STACK_SIZE_TOF = 8192;
static constexpr uint32_t STACK_SIZE_POWER = 4096;
static constexpr uint32_t STACK_SIZE_LED = 4096;
static constexpr uint32_t STACK_SIZE_BUTTON = 4096;
static constexpr uint32_t STACK_SIZE_COMM = 4096;
static constexpr uint32_t STACK_SIZE_CLI = 4096;
static constexpr uint32_t STACK_SIZE_TELEMETRY = 4096;

// =============================================================================
// Global Component Instances
// =============================================================================

// Global magnetometer calibrator (accessible from CLI)
stampfly::MagCalibrator* g_mag_calibrator = nullptr;

// Global logger pointer (accessible from CLI)
stampfly::Logger* g_logger_ptr = nullptr;

// Global controller comm pointer (accessible from CLI)
stampfly::ControllerComm* g_comm_ptr = nullptr;

// Global LED pointer (accessible from CLI)
stampfly::LED* g_led_ptr = nullptr;

// Global motor driver pointer (accessible from CLI)
stampfly::MotorDriver* g_motor_ptr = nullptr;

namespace {
    // Sensors
    stampfly::BMI270Wrapper g_imu;
    stampfly::BMM150 g_mag;
    stampfly::MagCalibrator g_mag_cal;  // Magnetometer calibrator instance
    stampfly::BMP280 g_baro;
    stampfly::VL53L3CXWrapper g_tof_bottom;
    stampfly::VL53L3CXWrapper g_tof_front;
    stampfly::PMW3901* g_optflow = nullptr;
    stampfly::PowerMonitor g_power;

    // Actuators
    stampfly::MotorDriver g_motor;
    stampfly::LED g_led;
    stampfly::Buzzer g_buzzer;
    stampfly::Button g_button;

    // Estimators
    stampfly::ESKF g_eskf;
    stampfly::AttitudeEstimator g_attitude_est;
    stampfly::AltitudeEstimator g_altitude_est;

    // Filters for IMU
    stampfly::LowPassFilter g_accel_lpf[3];
    stampfly::LowPassFilter g_gyro_lpf[3];

    // Communication
    stampfly::ControllerComm g_comm;

    // CLI
    stampfly::CLI g_cli;

    // Logger (400Hz binary log output)
    stampfly::Logger g_logger;

    // Barometer reference for relative altitude (ESKF expects NED, origin at boot)
    float g_baro_reference_altitude = 0.0f;
    bool g_baro_reference_set = false;

    // Mag reference ring buffer for averaging (1 second = 100 samples at 100Hz)
    static constexpr int MAG_REF_BUFFER_SIZE = 100;
    stampfly::math::Vector3 g_mag_buffer[MAG_REF_BUFFER_SIZE];
    int g_mag_buffer_index = 0;
    int g_mag_buffer_count = 0;
    bool g_mag_ref_set = false;  // mag_refが設定済みかどうか

    // 起動時のジャイロバイアス（binlog reset後に復元するため）
    stampfly::math::Vector3 g_initial_gyro_bias = stampfly::math::Vector3::zero();

    // ESKF準備完了フラグ（センサー安定・キャリブレーション完了後にtrue）
    volatile bool g_eskf_ready = false;

    // センサータスク正常動作フラグ（各タスクが有効なデータを取得したらtrue）
    volatile bool g_imu_task_healthy = false;
    volatile bool g_tof_task_healthy = false;
    volatile bool g_mag_task_healthy = false;
    volatile bool g_optflow_task_healthy = false;
    volatile bool g_baro_task_healthy = false;

    // センサ data_ready フラグ (ESKF updateをIMUTaskに集約するため)
    volatile bool g_mag_data_ready = false;
    volatile bool g_baro_data_ready = false;
    volatile bool g_tof_data_ready = false;

    // センサデータキャッシュ (data_ready時のデータを保持)
    stampfly::math::Vector3 g_mag_data_cache;
    float g_baro_data_cache = 0.0f;
    float g_tof_data_cache = 0.0f;

    // Task handles
    TaskHandle_t g_imu_task_handle = nullptr;
    TaskHandle_t g_control_task_handle = nullptr;
    TaskHandle_t g_optflow_task_handle = nullptr;
    TaskHandle_t g_mag_task_handle = nullptr;
    TaskHandle_t g_baro_task_handle = nullptr;
    TaskHandle_t g_tof_task_handle = nullptr;
    TaskHandle_t g_power_task_handle = nullptr;
    TaskHandle_t g_led_task_handle = nullptr;
    TaskHandle_t g_button_task_handle = nullptr;
    TaskHandle_t g_comm_task_handle = nullptr;
    TaskHandle_t g_cli_task_handle = nullptr;
    TaskHandle_t g_telemetry_task_handle = nullptr;

    // ESP Timer for precise 400Hz (2.5ms) IMU timing
    esp_timer_handle_t g_imu_timer = nullptr;
    SemaphoreHandle_t g_imu_semaphore = nullptr;
    SemaphoreHandle_t g_control_semaphore = nullptr;  // For control task synchronization

}

// デバッグ用: IMUタスクのチェックポイント（C言語からアクセス可能）
extern "C" {
    volatile uint8_t g_imu_checkpoint = 0;
    volatile uint32_t g_imu_last_loop = 0;
}

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Mag参照ベクトルをバッファの平均値で設定
 *
 * ARM時またはbinlog開始時に呼び出す
 */
static void setMagReferenceFromBuffer()
{
    if (g_mag_buffer_count == 0) {
        ESP_LOGW(TAG, "No mag samples in buffer, cannot set reference");
        return;
    }

    // バッファの平均を計算
    stampfly::math::Vector3 sum = stampfly::math::Vector3::zero();
    int count = std::min(g_mag_buffer_count, MAG_REF_BUFFER_SIZE);
    for (int i = 0; i < count; i++) {
        sum += g_mag_buffer[i];
    }
    stampfly::math::Vector3 avg = sum * (1.0f / count);

    // ESKFに設定
    if (g_eskf.isInitialized()) {
        g_eskf.setMagReference(avg);
        g_mag_ref_set = true;
        ESP_LOGI(TAG, "Mag reference set from %d samples: (%.1f, %.1f, %.1f) uT",
                 count, avg.x, avg.y, avg.z);
    }
}

/**
 * @brief binlog開始時のコールバック
 *
 * ESKFをリセットしてmag_refを設定する
 * PC版ESKFとの同期のため、binlog開始時に初期化
 */
static void onBinlogStart()
{
    // ESKFをリセット（PC版と同じ初期状態にする）
    if (g_eskf.isInitialized()) {
        g_eskf.reset();
        // ジャイロバイアスを復元（reset()でゼロになるため）
        g_eskf.setGyroBias(g_initial_gyro_bias);
        ESP_LOGI(TAG, "ESKF reset for binlog, gyro bias restored");
    }

    // mag_refを設定（バッファの最新値で更新）
    setMagReferenceFromBuffer();
}

// =============================================================================
// Timer Callbacks
// =============================================================================

/**
 * @brief ESP Timer callback for 400Hz IMU timing
 *
 * This callback runs from the esp_timer task context and gives a semaphore
 * to wake up the IMU task with precise 2.5ms timing.
 *
 * Note: ESP_TIMER_TASK dispatch runs in a high-priority FreeRTOS task,
 * not in ISR context, so we use xSemaphoreGive instead of xSemaphoreGiveFromISR.
 */
static void imu_timer_callback(void* arg)
{
    static uint32_t timer_count = 0;
    static uint32_t last_imu_loop = 0;
    timer_count++;

    // 10秒ごと（4000回）にタイマー生存確認
    if (timer_count % 4000 == 0) {
        // IMUタスクが進行しているかチェック
        if (g_imu_last_loop == last_imu_loop) {
            // IMUタスクが停止している
            ESP_LOGW(TAG, "IMU timer: count=%lu, IMU STUCK at checkpoint=%u, loop=%lu",
                     timer_count, g_imu_checkpoint, g_imu_last_loop);
        } else {
            ESP_LOGI(TAG, "IMU timer alive: count=%lu, imu_loop=%lu",
                     timer_count, g_imu_last_loop);
        }
        last_imu_loop = g_imu_last_loop;
    }
    xSemaphoreGive(g_imu_semaphore);
}

// =============================================================================
// Task Functions
// =============================================================================

/**
 * @brief IMU Task - 400Hz (2.5ms period)
 * Reads BMI270 FIFO, applies filters, updates estimators
 *
 * Timing: Uses ESP Timer for precise 2.5ms (400Hz) period.
 * The timer callback gives a semaphore which this task waits on.
 */
static void IMUTask(void* pvParameters)
{
    ESP_LOGI(TAG, "IMUTask started (400Hz via ESP Timer)");

    auto& state = stampfly::StampFlyState::getInstance();

    static uint32_t imu_loop_counter = 0;
    static uint32_t imu_read_fail_counter = 0;
    static uint32_t imu_consecutive_success = 0;
    constexpr uint32_t IMU_HEALTHY_THRESHOLD = 10;  // 10連続成功でhealthy

    while (true) {
        g_imu_checkpoint = 0;  // セマフォ待ち中

        // Wait for timer semaphore (precise 2.5ms = 400Hz timing)
        if (xSemaphoreTake(g_imu_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        imu_loop_counter++;
        g_imu_last_loop = imu_loop_counter;  // タイマーコールバックで監視用
        g_imu_checkpoint = 1;  // ループ開始

        // 10秒ごとにIMUタスク生存確認
        if (imu_loop_counter % 4000 == 0) {
            ESP_LOGI(TAG, "IMUTask alive: loop=%lu, read_fails=%lu, stack_free=%u",
                     imu_loop_counter, imu_read_fail_counter,
                     (unsigned)uxTaskGetStackHighWaterMark(nullptr));
        }

        g_imu_checkpoint = 2;  // isInitialized チェック前

        if (g_imu.isInitialized()) {
            stampfly::AccelData accel;
            stampfly::GyroData gyro;

            g_imu_checkpoint = 3;  // IMU読み取り前
            if (g_imu.readSensorData(accel, gyro) == ESP_OK) {
                g_imu_checkpoint = 4;  // IMU読み取り成功
                // ヘルスフラグ更新: 連続成功でhealthy
                imu_read_fail_counter = 0;
                if (++imu_consecutive_success >= IMU_HEALTHY_THRESHOLD) {
                    g_imu_task_healthy = true;
                }

                // ============================================================
                // BMI270座標系 → 機体座標系(NED) 変換
                // 図より:
                //   BMI270のX → 機体Y (右方向)
                //   BMI270のY → 機体X (前方)
                //   BMI270のZ → 機体-Z (上向き、NEDでは下が正なので符号反転)
                // 変換式:
                //   機体X = センサY
                //   機体Y = センサX
                //   機体Z = -センサZ
                //
                // 単位変換:
                //   加速度: g → m/s² (×9.81)
                //   ジャイロ: rad/s (変換不要)
                // ============================================================
                constexpr float GRAVITY = 9.81f;
                float accel_body_x = accel.y * GRAVITY;   // 前方正 [m/s²]
                float accel_body_y = accel.x * GRAVITY;   // 右正 [m/s²]
                float accel_body_z = -accel.z * GRAVITY;  // 下正 (NED) [m/s²]

                float gyro_body_x = gyro.y;     // Roll rate [rad/s]
                float gyro_body_y = gyro.x;     // Pitch rate [rad/s]
                float gyro_body_z = -gyro.z;    // Yaw rate [rad/s]

                // Apply low-pass filters (機体座標系で)
                float filtered_accel[3] = {
                    g_accel_lpf[0].apply(accel_body_x),
                    g_accel_lpf[1].apply(accel_body_y),
                    g_accel_lpf[2].apply(accel_body_z)
                };
                float filtered_gyro[3] = {
                    g_gyro_lpf[0].apply(gyro_body_x),
                    g_gyro_lpf[1].apply(gyro_body_y),
                    g_gyro_lpf[2].apply(gyro_body_z)
                };

                // Update state
                stampfly::StateVector3 accel_vec(filtered_accel[0], filtered_accel[1], filtered_accel[2]);
                stampfly::StateVector3 gyro_vec(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]);
                state.updateIMU(accel_vec, gyro_vec);

                // Prepare vectors for estimators
                stampfly::math::Vector3 a(filtered_accel[0], filtered_accel[1], filtered_accel[2]);
                stampfly::math::Vector3 g(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]);

                g_imu_checkpoint = 10;  // ESKF更新前

                // Update ESKF predict step (400Hz)
                // g_eskf_ready: センサー安定・キャリブレーション完了後にtrue
                if (g_eskf.isInitialized() && g_eskf_ready) {
                    static uint32_t flow_update_counter = 0;
                    static uint32_t eskf_error_counter = 0;

                    g_imu_checkpoint = 11;  // ESKF入力チェック

                    // 入力値の事前チェック
                    bool eskf_ok = std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
                                   std::isfinite(g.x) && std::isfinite(g.y) && std::isfinite(g.z);

                    if (eskf_ok) {
                        g_imu_checkpoint = 12;  // ESKF predict前
                        g_eskf.predict(a, g, 0.0025f);  // 2.5ms (400Hz)

                        g_imu_checkpoint = 13;  // updateAccelAttitude前
                        // 加速度計による姿勢補正 (400Hz)
                        g_eskf.updateAccelAttitude(a);

                        g_imu_checkpoint = 14;  // フロー更新セクション

                        // オプティカルフロー更新（100Hz = 400Hz / 4）
                        // ヘルスチェック: Flow + ToF が両方healthy必要（距離スケーリングに必要）
                        flow_update_counter++;
                        if (flow_update_counter >= 4) {
                            flow_update_counter = 0;
                            if (g_optflow_task_healthy && g_tof_task_healthy) {
                                int16_t flow_dx, flow_dy;
                                uint8_t flow_squal;
                                state.getFlowRawData(flow_dx, flow_dy, flow_squal);

                                if (stampfly::OutlierDetector::isFlowValid(flow_squal)) {
                                    float tof_bottom, tof_front;
                                    state.getToFData(tof_bottom, tof_front);
                                    float distance = tof_bottom;
                                    if (distance < 0.02f) distance = 0.02f;
                                    if (distance > 0.02f && distance < 4.0f) {
                                        // 新API: 生カウントとdt、機体ジャイロを渡す
                                        // ESKF内部で物理的に正しい変換を行う
                                        constexpr float dt = 0.01f;  // 100Hz (フローセンサーレート)
                                        g_eskf.updateFlowRaw(flow_dx, flow_dy, distance, dt, g.x, g.y);
                                    }
                                }
                            }
                        }

                        g_imu_checkpoint = 15;  // Baro更新セクション

                        // Baro更新（data_readyフラグで制御、50Hz）
                        // ヘルスチェック: Baro healthy必要
                        // TODO: 気圧センサの値が確認できたら有効化
                        if (g_baro_data_ready) {
                            g_baro_data_ready = false;
                            if (g_baro_task_healthy) {
                                // g_eskf.updateBaro(g_baro_data_cache);
                            }
                        }

                        g_imu_checkpoint = 16;  // ToF更新セクション

                        // ToF更新（data_readyフラグで制御、30Hz）
                        // ヘルスチェック: ToF healthy必要
                        if (g_tof_data_ready) {
                            g_tof_data_ready = false;
                            if (g_tof_task_healthy && g_tof_data_cache > 0.01f && g_tof_data_cache < 4.0f) {
                                g_eskf.updateToF(g_tof_data_cache);
                            }
                        }

                        g_imu_checkpoint = 17;  // Mag更新セクション

                        // Mag更新（data_readyフラグで制御、10Hz）
                        // ヘルスチェック: Mag healthy必要
                        if (g_mag_data_ready && g_mag_ref_set) {
                            g_mag_data_ready = false;
                            if (g_mag_task_healthy) {
                                g_eskf.updateMag(g_mag_data_cache);
                            }
                        }

                        g_imu_checkpoint = 20;  // getState前

                        // Update StampFlyState with ESKF estimated state
                        auto eskf_state = g_eskf.getState();

                        g_imu_checkpoint = 21;  // getState後、検証前

                        // 出力値の検証（NaNチェック + 発散検出）
                        bool eskf_valid = std::isfinite(eskf_state.roll) && std::isfinite(eskf_state.pitch);

                        // 発散検出: 位置/速度が異常に大きい場合
                        constexpr float MAX_POS = 100.0f;   // 100m以上は異常
                        constexpr float MAX_VEL = 50.0f;    // 50m/s以上は異常
                        bool pos_diverged = std::abs(eskf_state.position.x) > MAX_POS ||
                                           std::abs(eskf_state.position.y) > MAX_POS ||
                                           std::abs(eskf_state.position.z) > MAX_POS;
                        bool vel_diverged = std::abs(eskf_state.velocity.x) > MAX_VEL ||
                                           std::abs(eskf_state.velocity.y) > MAX_VEL ||
                                           std::abs(eskf_state.velocity.z) > MAX_VEL;

                        if (eskf_valid && !pos_diverged && !vel_diverged) {
                            g_imu_checkpoint = 22;  // state更新前
                            state.updateAttitude(eskf_state.roll, eskf_state.pitch, eskf_state.yaw);
                            state.updateEstimatedPosition(eskf_state.position.x, eskf_state.position.y, eskf_state.position.z);
                            state.updateEstimatedVelocity(eskf_state.velocity.x, eskf_state.velocity.y, eskf_state.velocity.z);
                            state.updateGyroBias(eskf_state.gyro_bias.x, eskf_state.gyro_bias.y, eskf_state.gyro_bias.z);
                            state.updateAccelBias(eskf_state.accel_bias.x, eskf_state.accel_bias.y, eskf_state.accel_bias.z);

                            g_imu_checkpoint = 23;  // state更新後、ロギング前

                            // === Binary logging (400Hz) ===
                            if (g_logger.isRunning()) {
                                stampfly::LogPacket pkt;
                                pkt.header[0] = 0xAA;
                                pkt.header[1] = 0x56;  // V2
                                pkt.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                                // IMU data (filtered, body frame)
                                pkt.accel_x = filtered_accel[0];
                                pkt.accel_y = filtered_accel[1];
                                pkt.accel_z = filtered_accel[2];
                                pkt.gyro_x = filtered_gyro[0];
                                pkt.gyro_y = filtered_gyro[1];
                                pkt.gyro_z = filtered_gyro[2];

                                // Mag data (from state cache)
                                stampfly::Vec3 mag_cached;
                                state.getMagData(mag_cached);
                                pkt.mag_x = mag_cached.x;
                                pkt.mag_y = mag_cached.y;
                                pkt.mag_z = mag_cached.z;

                                // Baro data
                                float baro_alt_cached, pressure_cached;
                                state.getBaroData(baro_alt_cached, pressure_cached);
                                pkt.pressure = pressure_cached;
                                pkt.baro_alt = baro_alt_cached;

                                // ToF data
                                float tof_bottom_cached, tof_front_cached;
                                state.getToFData(tof_bottom_cached, tof_front_cached);
                                pkt.tof_bottom = tof_bottom_cached;
                                pkt.tof_front = tof_front_cached;

                                // OptFlow raw data
                                int16_t flow_dx_cached, flow_dy_cached;
                                uint8_t flow_squal_cached;
                                state.getFlowRawData(flow_dx_cached, flow_dy_cached, flow_squal_cached);
                                pkt.flow_dx = flow_dx_cached;
                                pkt.flow_dy = flow_dy_cached;
                                pkt.flow_squal = flow_squal_cached;

                                // ESKF estimates
                                pkt.pos_x = eskf_state.position.x;
                                pkt.pos_y = eskf_state.position.y;
                                pkt.pos_z = eskf_state.position.z;
                                pkt.vel_x = eskf_state.velocity.x;
                                pkt.vel_y = eskf_state.velocity.y;
                                pkt.vel_z = eskf_state.velocity.z;
                                pkt.roll = eskf_state.roll;
                                pkt.pitch = eskf_state.pitch;
                                pkt.yaw = eskf_state.yaw;
                                pkt.gyro_bias_z = eskf_state.gyro_bias.z;
                                pkt.accel_bias_x = eskf_state.accel_bias.x;
                                pkt.accel_bias_y = eskf_state.accel_bias.y;

                                // Status
                                pkt.eskf_status = 1;  // running
                                pkt.baro_ref_alt = g_baro_reference_altitude;
                                memset(pkt.reserved, 0, sizeof(pkt.reserved));
                                pkt.checksum = 0;  // Will be calculated by Logger

                                g_logger.pushData(pkt);
                            }
                            g_imu_checkpoint = 24;  // ロギング完了
                        } else {
                            eskf_error_counter++;

                            // 発散時は常にリセット（binlog onと同じ処理）
                            if (pos_diverged || vel_diverged) {
                                g_eskf.reset();
                                g_eskf.setGyroBias(g_initial_gyro_bias);
                                setMagReferenceFromBuffer();
                            }

                            // ログ出力は100回に1回（スパム防止）
                            if (eskf_error_counter % 100 == 1) {
                                if (!eskf_valid) {
                                    ESP_LOGW(TAG, "ESKF output NaN, errors=%lu", eskf_error_counter);
                                } else {
                                    ESP_LOGW(TAG, "ESKF diverged: pos=[%.1f,%.1f,%.1f] vel=[%.1f,%.1f,%.1f]",
                                             eskf_state.position.x, eskf_state.position.y, eskf_state.position.z,
                                             eskf_state.velocity.x, eskf_state.velocity.y, eskf_state.velocity.z);
                                }
                            }
                        }
                    } else {
                        eskf_error_counter++;
                    }
                }
                // Fallback to simple attitude estimator if ESKF not available
                else if (g_attitude_est.isInitialized()) {
                    g_attitude_est.update(a, g, 0.0025f);
                    auto att_state = g_attitude_est.getState();
                    state.updateAttitude(att_state.roll, att_state.pitch, att_state.yaw);
                }

                g_imu_checkpoint = 30;  // ControlTask起動前

                // Wake up ControlTask (runs at same 400Hz rate)
                xSemaphoreGive(g_control_semaphore);

                g_imu_checkpoint = 31;  // ControlTask起動後
            } else {
                // ヘルスフラグ更新: 連続失敗でunhealthy
                imu_consecutive_success = 0;
                if (++imu_read_fail_counter >= 3) {
                    g_imu_task_healthy = false;
                }
                // 連続失敗時にログ出力
                if (imu_read_fail_counter % 400 == 1) {
                    ESP_LOGW(TAG, "IMU read failed, consecutive fails=%lu", imu_read_fail_counter);
                }
            }
        }

        g_imu_checkpoint = 99;  // ループ完了（次のセマフォ待ちへ）
        // No delay here - timing controlled by ESP Timer semaphore
    }
}

/**
 * @brief Control Task - 400Hz (2.5ms period)
 *
 * This task handles flight control (attitude/position control, motor mixing).
 * It runs at 400Hz, synchronized with IMU updates via semaphore.
 *
 * ============================================================================
 * STUB IMPLEMENTATION - Replace with your flight control code
 * ============================================================================
 *
 * Typical flight control loop:
 * 1. Read current state (attitude, position, velocity) from StampFlyState
 * 2. Read control inputs (throttle, roll, pitch, yaw commands)
 * 3. Compute control outputs using PID or other control algorithms
 * 4. Apply motor mixing for X-quad configuration
 * 5. Send PWM commands to motors
 *
 * See docs/developer_guide.md for detailed PID control example.
 */
static void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started (400Hz via semaphore)");

    auto& state = stampfly::StampFlyState::getInstance();

    // ========================================================================
    // Motor Layout (X-quad configuration, viewed from above)
    // ========================================================================
    //
    //               Front
    //          FL (M4)   FR (M1)
    //             ╲   ▲   ╱
    //              ╲  │  ╱
    //               ╲ │ ╱
    //                ╲│╱
    //                 ╳         ← Center of drone
    //                ╱│╲
    //               ╱ │ ╲
    //              ╱  │  ╲
    //             ╱   │   ╲
    //          RL (M3)    RR (M2)
    //                Rear
    //
    // Motor rotation:
    //   M1 (FR): CCW (Counter-Clockwise)
    //   M2 (RR): CW  (Clockwise)
    //   M3 (RL): CCW (Counter-Clockwise)
    //   M4 (FL): CW  (Clockwise)
    //
    // ========================================================================

    while (true) {
        // Wait for control semaphore (given by IMU task after ESKF update)
        if (xSemaphoreTake(g_control_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Get current flight state
        stampfly::FlightState flight_state = state.getFlightState();

        // Only run control when ARMED or FLYING
        if (flight_state != stampfly::FlightState::ARMED &&
            flight_state != stampfly::FlightState::FLYING) {
            // Ensure motors are stopped when not armed
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_FR, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_RR, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_RL, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_FL, 0);
            continue;
        }

        // Get control input from controller
        // throttle: 0.0 ~ 1.0
        // roll, pitch, yaw: -1.0 ~ +1.0 (not used in this simple example)
        float throttle, roll, pitch, yaw;
        state.getControlInput(throttle, roll, pitch, yaw);

        // Simple throttle control: all motors receive same throttle value
        // TODO: Add attitude control (PID) and motor mixing for stable flight
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_FR, throttle);  // M1
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_RR, throttle);  // M2
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_RL, throttle);  // M3
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_FL, throttle);  // M4
    }
}

/**
 * @brief Optical Flow Task - 100Hz (10ms period)
 */
static void OptFlowTask(void* pvParameters)
{
    ESP_LOGI(TAG, "OptFlowTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    auto& state = stampfly::StampFlyState::getInstance();

    // ヘルスチェック用カウンタ
    int consecutive_valid = 0;
    int consecutive_invalid = 0;
    constexpr int HEALTHY_THRESHOLD = 10;   // 10連続成功でhealthy (100ms)
    constexpr int UNHEALTHY_THRESHOLD = 10; // 10連続失敗でunhealthy (100ms)

    while (true) {
        if (g_optflow != nullptr) {
            try {
                auto burst = g_optflow->readMotionBurst();
                // Check quality
                if (stampfly::OutlierDetector::isFlowValid(burst.squal)) {
                    // ヘルスフラグ更新: 連続成功でhealthy
                    consecutive_invalid = 0;
                    if (++consecutive_valid >= HEALTHY_THRESHOLD) {
                        g_optflow_task_healthy = true;
                    }

                    // ============================================================
                    // PMW3901座標系 → 中間座標系 変換（第1段階）
                    // この変換はbinlog出力と互換性を保つためのもの
                    // 最終的な機体座標への変換はESKF内部で行われる
                    // ============================================================
                    int16_t flow_body_x = -burst.delta_y;
                    int16_t flow_body_y =  burst.delta_x;

                    state.updateOpticalFlow(flow_body_x, flow_body_y, burst.squal);
                    // ESKF updateはIMUTask内で行う（レースコンディション防止）
                } else {
                    // 品質不良
                    consecutive_valid = 0;
                    if (++consecutive_invalid >= UNHEALTHY_THRESHOLD) {
                        g_optflow_task_healthy = false;
                    }
                }
            } catch (const stampfly::PMW3901Exception& e) {
                // Sensor read error
                consecutive_valid = 0;
                if (++consecutive_invalid >= UNHEALTHY_THRESHOLD) {
                    g_optflow_task_healthy = false;
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Magnetometer Task - 100Hz (10ms period)
 */
static void MagTask(void* pvParameters)
{
    ESP_LOGI(TAG, "MagTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz センサ読み取り

    auto& state = stampfly::StampFlyState::getInstance();

    // ESKF更新は10Hz（PC版と同じ）
    int eskf_update_counter = 0;
    constexpr int ESKF_UPDATE_DIVISOR = 10;  // 100Hz / 10 = 10Hz

    // ヘルスチェック用カウンタ
    int consecutive_success = 0;
    int consecutive_fail = 0;
    constexpr int HEALTHY_THRESHOLD = 10;   // 10連続成功でhealthy (100ms)
    constexpr int UNHEALTHY_THRESHOLD = 10; // 10連続失敗でunhealthy (100ms)

    while (true) {
        if (g_mag.isInitialized()) {
            stampfly::MagData mag;
            if (g_mag.read(mag) == ESP_OK) {
                // ヘルスフラグ更新: 連続成功でhealthy
                consecutive_fail = 0;
                if (++consecutive_success >= HEALTHY_THRESHOLD) {
                    g_mag_task_healthy = true;
                }
                // ============================================================
                // BMM150座標系 → 機体座標系(NED) 変換
                // 実測により確認:
                //   機体X = -センサY
                //   機体Y = センサX
                //   機体Z = センサZ
                // ============================================================
                float mag_body_x = -mag.y;  // 前方正
                float mag_body_y = mag.x;   // 右正
                float mag_body_z = mag.z;   // 下正 (NED)

                // キャリブレーションデータ収集中の場合
                if (g_mag_cal.getState() == stampfly::MagCalibrator::State::COLLECTING) {
                    g_mag_cal.addSample(mag_body_x, mag_body_y, mag_body_z);
                }

                // キャリブレーション適用
                float cal_mag_x, cal_mag_y, cal_mag_z;
                if (g_mag_cal.isCalibrated()) {
                    g_mag_cal.applyCalibration(mag_body_x, mag_body_y, mag_body_z,
                                                cal_mag_x, cal_mag_y, cal_mag_z);
                } else {
                    // キャリブレーション未実施の場合は生データをそのまま使用
                    cal_mag_x = mag_body_x;
                    cal_mag_y = mag_body_y;
                    cal_mag_z = mag_body_z;
                }

                // Update state with calibrated data
                state.updateMag(cal_mag_x, cal_mag_y, cal_mag_z);

                // キャリブレーション済みの場合のみESKF用データを準備
                if (g_mag_cal.isCalibrated()) {
                    stampfly::math::Vector3 m(cal_mag_x, cal_mag_y, cal_mag_z);

                    // リングバッファに追加（ARM/binlog開始時の平均計算用）
                    g_mag_buffer[g_mag_buffer_index] = m;
                    g_mag_buffer_index = (g_mag_buffer_index + 1) % MAG_REF_BUFFER_SIZE;
                    if (g_mag_buffer_count < MAG_REF_BUFFER_SIZE) {
                        g_mag_buffer_count++;
                    }

                    // ESKF用データをキャッシュしてフラグを立てる（10Hz）
                    // ESKF updateはIMUTask内で行う（レースコンディション防止）
                    eskf_update_counter++;
                    if (eskf_update_counter >= ESKF_UPDATE_DIVISOR) {
                        eskf_update_counter = 0;
                        g_mag_data_cache = m;
                        g_mag_data_ready = true;
                    }
                }
            } else {
                // ヘルスフラグ更新: 連続失敗でunhealthy
                consecutive_success = 0;
                if (++consecutive_fail >= UNHEALTHY_THRESHOLD) {
                    g_mag_task_healthy = false;
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Barometer Task - 50Hz (20ms period)
 */
static void BaroTask(void* pvParameters)
{
    ESP_LOGI(TAG, "BaroTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    auto& state = stampfly::StampFlyState::getInstance();

    // ヘルスチェック用カウンタ
    int consecutive_success = 0;
    int consecutive_fail = 0;
    constexpr int HEALTHY_THRESHOLD = 5;   // 5連続成功でhealthy (100ms)
    constexpr int UNHEALTHY_THRESHOLD = 10; // 10連続失敗でunhealthy (200ms)

    while (true) {
        if (g_baro.isInitialized()) {
            stampfly::BaroData baro;
            if (g_baro.read(baro) == ESP_OK) {
                // ヘルスフラグ更新: 連続成功でhealthy
                consecutive_fail = 0;
                if (++consecutive_success >= HEALTHY_THRESHOLD) {
                    g_baro_task_healthy = true;
                }
                // Use altitude from read() directly (already calculated)
                state.updateBaro(baro.pressure_pa, baro.temperature_c, baro.altitude_m);

                // 初回測定で基準高度を設定
                if (!g_baro_reference_set) {
                    g_baro_reference_altitude = baro.altitude_m;
                    g_baro_reference_set = true;
                    state.setBaroReferenceAltitude(g_baro_reference_altitude);
                    ESP_LOGI(TAG, "Baro reference set: %.3f m", g_baro_reference_altitude);
                }

                // ESKF用データをキャッシュしてフラグを立てる（50Hz）
                // ESKF updateはIMUTask内で行う（レースコンディション防止）
                if (g_baro_reference_set) {
                    float relative_alt = baro.altitude_m - g_baro_reference_altitude;
                    g_baro_data_cache = relative_alt;
                    g_baro_data_ready = true;
                }

                // Fallback to simple altitude estimator (ESKF未使用時)
                if (!g_eskf.isInitialized() && g_altitude_est.isInitialized()) {
                    g_altitude_est.updateBaro(baro.altitude_m);
                }
            } else {
                // ヘルスフラグ更新: 連続失敗でunhealthy
                consecutive_success = 0;
                if (++consecutive_fail >= UNHEALTHY_THRESHOLD) {
                    g_baro_task_healthy = false;
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief ToF Task - 30Hz (33ms period)
 */
static void ToFTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ToFTask started, bottom_init=%d, front_init=%d",
             g_tof_bottom.isInitialized(), g_tof_front.isInitialized());

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // ~30Hz

    auto& state = stampfly::StampFlyState::getInstance();

    // Error counters for sensor disable on repeated failures
    const int MAX_ERRORS = 10;
    // ヘルスチェック用カウンタ (Bottom ToFのみ、ESKFで使用)
    int tof_consecutive_valid = 0;
    int tof_consecutive_invalid = 0;  // 無効測定の連続カウンタ
    constexpr int TOF_HEALTHY_THRESHOLD = 5;    // 5連続成功でhealthy (~165ms)
    constexpr int TOF_UNHEALTHY_THRESHOLD = 10; // 10連続無効でunhealthy (~330ms)
    // 距離の急激な変化検出用
    float tof_last_valid_distance = 0.0f;
    constexpr float TOF_MAX_CHANGE_RATE = 2.0f;  // 最大変化率 [m/s]（30Hz想定で1回あたり約6.7cm）
    constexpr float TOF_MAX_CHANGE_PER_CYCLE = TOF_MAX_CHANGE_RATE / 30.0f;  // 約0.067m
    int bottom_errors = 0;
    int front_errors = 0;
    bool bottom_disabled = false;
    bool front_disabled = false;

    static int log_count = 0;

    while (true) {
        // Bottom ToF
        if (g_tof_bottom.isInitialized() && !bottom_disabled) {
            // Check if data is ready
            bool data_ready = false;
            g_tof_bottom.isDataReady(data_ready);

            if (data_ready) {
                uint16_t distance_mm;
                uint8_t status;
                esp_err_t ret = g_tof_bottom.getDistance(distance_mm, status);
                if (ret == ESP_OK) {
                    bottom_errors = 0;  // Reset on success

                    // Only update if valid measurement (status 0-4)
                    if (status <= 4) {
                        float distance_m = distance_mm * 0.001f;

                        // 距離の急激な変化をチェック
                        bool distance_jump_detected = false;
                        if (tof_last_valid_distance > 0.01f) {
                            float change = std::abs(distance_m - tof_last_valid_distance);
                            if (change > TOF_MAX_CHANGE_PER_CYCLE) {
                                distance_jump_detected = true;
                                ESP_LOGW(TAG, "ToF distance jump: %.3f -> %.3f (change=%.3f)",
                                         tof_last_valid_distance, distance_m, change);
                            }
                        }

                        state.updateToF(stampfly::ToFPosition::BOTTOM, distance_m, status);

                        if (!distance_jump_detected) {
                            // ヘルスフラグ更新: 有効な測定でhealthy
                            tof_consecutive_invalid = 0;
                            if (++tof_consecutive_valid >= TOF_HEALTHY_THRESHOLD) {
                                g_tof_task_healthy = true;
                            }

                            // ESKF用データをキャッシュしてフラグを立てる（30Hz）
                            // ESKF updateはIMUTask内で行う（レースコンディション防止）
                            g_tof_data_cache = distance_m;
                            g_tof_data_ready = true;

                            // Fallback to simple altitude estimator (ESKF未使用時)
                            if (!g_eskf.isInitialized() && g_altitude_est.isInitialized() && g_attitude_est.isInitialized()) {
                                auto att = g_attitude_est.getState();
                                g_altitude_est.updateToF(distance_m, att.pitch, att.roll);
                            }
                        } else {
                            // 距離ジャンプ検出: unhealthy判定
                            tof_consecutive_valid = 0;
                            if (++tof_consecutive_invalid >= TOF_UNHEALTHY_THRESHOLD) {
                                g_tof_task_healthy = false;
                            }
                        }

                        // 有効距離を記録（ジャンプ後も更新して追従可能に）
                        tof_last_valid_distance = distance_m;
                    } else {
                        // ステータス異常: unhealthy判定カウント
                        tof_consecutive_valid = 0;
                        if (++tof_consecutive_invalid >= TOF_UNHEALTHY_THRESHOLD) {
                            g_tof_task_healthy = false;
                        }
                    }

                    // Debug log every 30 readings (~1 second)
                    if (++log_count >= 30) {
                        ESP_LOGI(TAG, "ToF Bottom: %d mm, status=%d", distance_mm, status);
                        log_count = 0;
                    }

                    // Clear interrupt and start next measurement
                    g_tof_bottom.clearInterruptAndStartMeasurement();
                } else {
                    // ヘルスフラグ更新: エラーでunhealthy
                    tof_consecutive_valid = 0;
                    if (++bottom_errors >= MAX_ERRORS) {
                        g_tof_task_healthy = false;
                        ESP_LOGW(TAG, "Bottom ToF disabled: err=%s", esp_err_to_name(ret));
                        bottom_disabled = true;
                    }
                }
            }
        }

        // Front ToF
        if (g_tof_front.isInitialized() && !front_disabled) {
            bool data_ready = false;
            g_tof_front.isDataReady(data_ready);

            if (data_ready) {
                uint16_t distance_mm;
                uint8_t status;
                if (g_tof_front.getDistance(distance_mm, status) == ESP_OK) {
                    front_errors = 0;  // Reset on success
                    if (status <= 4) {
                        float distance_m = distance_mm * 0.001f;
                        state.updateToF(stampfly::ToFPosition::FRONT, distance_m, status);
                    }
                    g_tof_front.clearInterruptAndStartMeasurement();
                } else {
                    if (++front_errors >= MAX_ERRORS) {
                        ESP_LOGW(TAG, "Front ToF disabled due to repeated errors");
                        front_disabled = true;
                    }
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Power Monitor Task - 10Hz (100ms period)
 */
static void PowerTask(void* pvParameters)
{
    ESP_LOGI(TAG, "PowerTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz

    auto& state = stampfly::StampFlyState::getInstance();
    static uint32_t log_counter = 0;
    static bool first_read = true;
    static bool low_battery_warned = false;

    while (true) {
        if (g_power.isInitialized()) {
            stampfly::PowerData power;
            if (g_power.read(power) == ESP_OK) {
                state.updatePower(power.voltage_v, power.current_ma / 1000.0f);

                // Log first reading immediately, then every 5 seconds
                if (first_read || ++log_counter >= 50) {
                    ESP_LOGI(TAG, "Battery: %.2fV, %.1fmA, LowBat=%d",
                             power.voltage_v, power.current_ma, g_power.isLowBattery());
                    log_counter = 0;
                    first_read = false;
                }

                // Low battery warning (only warn once to avoid continuous buzzing)
                if (g_power.isLowBattery() && !low_battery_warned) {
                    ESP_LOGW(TAG, "LOW BATTERY WARNING: %.2fV", power.voltage_v);
                    state.setError(stampfly::ErrorCode::LOW_BATTERY);
                    g_led.setPattern(stampfly::LED::Pattern::BLINK_FAST, 0xFF0000);
                    g_buzzer.lowBatteryWarning();
                    low_battery_warned = true;
                }
                // Reset warning flag when battery is charged again
                if (!g_power.isLowBattery()) {
                    low_battery_warned = false;
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief LED Task - 30Hz (32ms period)
 */
static void LEDTask(void* pvParameters)
{
    ESP_LOGI(TAG, "LEDTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(32);  // ~30Hz

    // Low battery threshold for battery replace warning (cyan)
    constexpr float LOW_BATTERY_THRESHOLD = 3.4f;

    auto& state = stampfly::StampFlyState::getInstance();
    stampfly::FlightState prev_flight_state = stampfly::FlightState::INIT;
    bool prev_low_battery = false;

    while (true) {
        // Check for low battery (< 3.4V = battery replace warning)
        float voltage = state.getVoltage();
        bool low_battery = (voltage > 0.5f) && (voltage < LOW_BATTERY_THRESHOLD);

        // Update LED pattern based on flight state
        stampfly::FlightState flight_state = state.getFlightState();

        // Low battery takes priority over flight state
        if (low_battery != prev_low_battery || flight_state != prev_flight_state) {
            if (low_battery) {
                // Battery replace warning - cyan blink
                g_led.showLowBatteryCyan();
            } else {
                switch (flight_state) {
                    case stampfly::FlightState::INIT:
                        g_led.showInit();
                        break;
                    case stampfly::FlightState::CALIBRATING:
                        g_led.showCalibrating();
                        break;
                    case stampfly::FlightState::IDLE:
                        g_led.showIdle();
                        break;
                    case stampfly::FlightState::ARMED:
                        g_led.showArmed();
                        break;
                    case stampfly::FlightState::FLYING:
                        g_led.showFlying();
                        break;
                    case stampfly::FlightState::LANDING:
                        g_led.showLanding();
                        break;
                    case stampfly::FlightState::ERROR:
                        g_led.showError();
                        break;
                }
            }
            prev_flight_state = flight_state;
            prev_low_battery = low_battery;
        }

        // Update LED animation
        g_led.update();

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Button Task - 100Hz (10ms period)
 */
static void ButtonTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ButtonTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    while (true) {
        if (g_button.isInitialized()) {
            g_button.tick();
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Communication Task - 50Hz (20ms period)
 */
static void CommTask(void* pvParameters)
{
    ESP_LOGI(TAG, "CommTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    auto& state = stampfly::StampFlyState::getInstance();

    while (true) {
        if (g_comm.isInitialized()) {
            // Check connection timeout
            g_comm.tick();

            // Send telemetry if connected
            if (g_comm.isConnected()) {
                stampfly::TelemetryPacket telem = {};

                // Fill telemetry data
                telem.battery_mv = static_cast<uint16_t>(state.getVoltage() * 1000);
                telem.altitude_cm = static_cast<int16_t>(state.getAltitude() * 100);

                stampfly::StateVector3 vel = state.getVelocity();
                telem.velocity_x = static_cast<int16_t>(vel.x * 1000);
                telem.velocity_y = static_cast<int16_t>(vel.y * 1000);
                telem.velocity_z = static_cast<int16_t>(vel.z * 1000);

                stampfly::StateVector3 att = state.getAttitude();
                telem.roll_deg10 = static_cast<int16_t>(att.x * 180.0f / M_PI * 10);
                telem.pitch_deg10 = static_cast<int16_t>(att.y * 180.0f / M_PI * 10);
                telem.yaw_deg10 = static_cast<int16_t>(att.z * 180.0f / M_PI * 10);

                telem.state = static_cast<uint8_t>(state.getFlightState());

                // Set warning flags
                telem.flags = 0;
                if (g_power.isLowBattery()) {
                    telem.flags |= stampfly::TELEM_FLAG_LOW_BATTERY;
                }
                if (state.getErrorCode() != stampfly::ErrorCode::NONE) {
                    telem.flags |= stampfly::TELEM_FLAG_SENSOR_ERROR;
                }
                if (state.getFlightState() == stampfly::FlightState::CALIBRATING) {
                    telem.flags |= stampfly::TELEM_FLAG_CALIBRATING;
                }

                g_comm.sendTelemetry(telem);
            }

            // Update connection state
            if (!g_comm.isConnected() && state.getPairingState() == stampfly::PairingState::PAIRED) {
                state.setPairingState(stampfly::PairingState::PAIRED);  // Keep paired but disconnected
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief CLI Task - Low priority, processes USB serial input
 *
 * Note: Binary logging moved to stampfly_logger component (400Hz via ESP Timer)
 */
static void CLITask(void* pvParameters)
{
    ESP_LOGI(TAG, "CLITask started");

    // Print initial prompt
    g_cli.print("\r\n=== StampFly RTOS Skeleton ===\r\n");
    g_cli.print("Type 'help' for available commands\r\n");
    g_cli.print("> ");

    TickType_t last_teleplot = xTaskGetTickCount();
    TickType_t last_csvlog = xTaskGetTickCount();
    const TickType_t teleplot_period = pdMS_TO_TICKS(50);  // 20Hz teleplot output
    const TickType_t csvlog_period = pdMS_TO_TICKS(50);    // 20Hz CSV log output

    while (true) {
        if (g_cli.isInitialized()) {
            g_cli.processInput();

            TickType_t now = xTaskGetTickCount();

            // Output teleplot data at fixed interval
            if (g_cli.isTeleplotEnabled() && (now - last_teleplot) >= teleplot_period) {
                g_cli.outputTeleplot();
                last_teleplot = now;
            }

            // Output CSV log data at fixed interval
            if (g_cli.isLogEnabled() && (now - last_csvlog) >= csvlog_period) {
                g_cli.outputCSVLog();
                last_csvlog = now;
            }

            // Binary logging now handled by stampfly_logger component at 400Hz
            // controlled via g_logger.start()/stop()
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms polling (teleplot/csvlog only)
    }
}

// =============================================================================
// Telemetry Task - WebSocket data broadcast at 50Hz
// =============================================================================

static void TelemetryTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TelemetryTask started");

    auto& telemetry = stampfly::Telemetry::getInstance();
    auto& state = stampfly::StampFlyState::getInstance();

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    // DEBUG: ESP32側送信カウンタ
    static uint32_t esp32_send_counter = 0;

    while (true) {
        // Only broadcast when clients are connected
        if (telemetry.hasClients()) {
            stampfly::TelemetryWSPacket pkt = {};
            pkt.header = 0xAA;
            pkt.packet_type = 0x10;
            pkt.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

            // Get attitude from state (全角度送信)
            state.getAttitudeEuler(pkt.roll, pkt.pitch, pkt.yaw);

            // Get position and velocity
            auto pos = state.getPosition();
            pkt.pos_x = pos.x;
            pkt.pos_y = pos.y;
            pkt.pos_z = pos.z;

            auto vel = state.getVelocity();
            pkt.vel_x = vel.x;
            pkt.vel_y = vel.y;
            pkt.vel_z = vel.z;

            // ハートビート（専用フィールド）
            esp32_send_counter++;
            pkt.heartbeat = esp32_send_counter;

            // Get battery voltage
            float voltage, current;
            state.getPowerData(voltage, current);
            pkt.voltage = voltage;

            // Get flight state
            pkt.flight_state = static_cast<uint8_t>(state.getFlightState());

            // Calculate checksum (XOR of all bytes)
            uint8_t checksum = 0;
            const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
            for (size_t i = 0; i < sizeof(pkt) - 1; i++) {
                checksum ^= data[i];
            }
            pkt.checksum = checksum;

            // Broadcast to all connected clients
            telemetry.broadcast(&pkt, sizeof(pkt));
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

// =============================================================================
// Button Event Handler
// =============================================================================

static void onButtonEvent(stampfly::Button::Event event)
{
    auto& state = stampfly::StampFlyState::getInstance();

    switch (event) {
        case stampfly::Button::Event::CLICK:
            ESP_LOGI(TAG, "Button: CLICK");
            // Toggle arm/disarm in IDLE state
            if (state.getFlightState() == stampfly::FlightState::IDLE) {
                if (state.requestArm()) {
                    // ARM時にmag_refを設定（現在の向き=Yaw 0°）
                    setMagReferenceFromBuffer();
                    g_buzzer.armTone();
                    ESP_LOGI(TAG, "Motors ARMED");
                }
            } else if (state.getFlightState() == stampfly::FlightState::ARMED) {
                if (state.requestDisarm()) {
                    g_buzzer.disarmTone();
                    ESP_LOGI(TAG, "Motors DISARMED");
                }
            }
            break;

        case stampfly::Button::Event::DOUBLE_CLICK:
            ESP_LOGI(TAG, "Button: DOUBLE_CLICK");
            break;

        case stampfly::Button::Event::LONG_PRESS_START:
            ESP_LOGI(TAG, "Button: LONG_PRESS_START");
            break;

        case stampfly::Button::Event::LONG_PRESS_3S:
            ESP_LOGI(TAG, "Button: LONG_PRESS (3s) - Entering pairing mode");
            g_comm.enterPairingMode();
            state.setPairingState(stampfly::PairingState::PAIRING);
            g_led.setPattern(stampfly::LED::Pattern::BLINK_FAST, 0x0000FF);  // Blue fast blink
            g_buzzer.beep();
            break;

        case stampfly::Button::Event::LONG_PRESS_5S:
            ESP_LOGI(TAG, "Button: LONG_PRESS (5s) - System reset");
            g_buzzer.beep();
            vTaskDelay(pdMS_TO_TICKS(600));
            esp_restart();
            break;

        default:
            break;
    }
}

// =============================================================================
// Control Packet Handler
// =============================================================================

static void onControlPacket(const stampfly::ControlPacket& packet)
{
    auto& state = stampfly::StampFlyState::getInstance();

    // Update control inputs (raw ADC values)
    state.updateControlInput(
        packet.throttle,
        packet.roll,
        packet.pitch,
        packet.yaw
    );

    // Update control flags
    state.updateControlFlags(packet.flags);

    // Handle arm/disarm toggle from controller (rising edge detection)
    static bool prev_arm_flag = false;
    bool arm_flag = (packet.flags & stampfly::CTRL_FLAG_ARM) != 0;

    // Detect rising edge (button press)
    if (arm_flag && !prev_arm_flag) {
        stampfly::FlightState flight_state = state.getFlightState();
        if (flight_state == stampfly::FlightState::IDLE ||
            flight_state == stampfly::FlightState::ERROR) {
            // IDLE/ERROR → ARM
            if (state.requestArm()) {
                g_motor.arm();  // Enable motor driver
                setMagReferenceFromBuffer();
                g_buzzer.armTone();
                ESP_LOGI(TAG, "Motors ARMED (from controller)");
            }
        } else if (flight_state == stampfly::FlightState::ARMED) {
            // ARMED → DISARM
            if (state.requestDisarm()) {
                g_motor.disarm();  // Disable motor driver and stop motors
                g_buzzer.disarmTone();
                ESP_LOGI(TAG, "Motors DISARMED (from controller)");
            }
        }
    }
    prev_arm_flag = arm_flag;
}

// =============================================================================
// Global I2C bus handle
// =============================================================================

static i2c_master_bus_handle_t g_i2c_bus = nullptr;

// =============================================================================
// Initialization Functions
// =============================================================================

static esp_err_t initI2C()
{
    ESP_LOGI(TAG, "Initializing I2C bus...");

    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = static_cast<gpio_num_t>(GPIO_I2C_SCL);
    bus_config.sda_io_num = static_cast<gpio_num_t>(GPIO_I2C_SDA);
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &g_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus initialized");
    return ESP_OK;
}

static esp_err_t initSensors()
{
    ESP_LOGI(TAG, "Initializing sensors...");
    esp_err_t ret;

    // IMU (BMI270) - SPI with default StampFly config
    {
        auto cfg = stampfly::BMI270Wrapper::Config::defaultStampFly();
        ret = g_imu.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "IMU initialized");
        }
    }

    // Magnetometer (BMM150) - I2C
    {
        stampfly::BMM150::Config cfg;
        cfg.i2c_bus = g_i2c_bus;
        cfg.i2c_addr = stampfly::BMM150_I2C_ADDR_DEFAULT;
        cfg.data_rate = stampfly::BMM150DataRate::ODR_10HZ;
        cfg.preset = stampfly::BMM150Preset::REGULAR;

        ret = g_mag.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Magnetometer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Magnetometer initialized");
        }
    }

    // Barometer (BMP280) - I2C
    {
        stampfly::BMP280::Config cfg;
        cfg.i2c_bus = g_i2c_bus;
        cfg.i2c_addr = stampfly::BMP280_I2C_ADDR_DEFAULT;
        cfg.mode = stampfly::BMP280Mode::NORMAL;
        cfg.press_os = stampfly::BMP280Oversampling::X4;
        cfg.temp_os = stampfly::BMP280Oversampling::X2;
        cfg.standby = stampfly::BMP280Standby::MS_62_5;
        cfg.filter = stampfly::BMP280Filter::COEF_4;

        ret = g_baro.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Barometer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Barometer initialized");
        }
    }

    // ToF sensors (VL53L3CX) - I2C with XSHUT control
    // Dual sensor initialization: bottom (altitude) and front (obstacle detection)
    // Note: Front ToF is optional (removable for battery adapter)
    {
        ret = stampfly::VL53L3CXWrapper::initDualSensors(
            g_tof_bottom,
            g_tof_front,
            g_i2c_bus,
            static_cast<gpio_num_t>(GPIO_TOF_XSHUT_BOTTOM),
            static_cast<gpio_num_t>(GPIO_TOF_XSHUT_FRONT)
        );
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ToF init failed: %s", esp_err_to_name(ret));
        } else {
            // Start ranging for initialized sensors
            if (g_tof_bottom.isInitialized()) {
                g_tof_bottom.startRanging();
                ESP_LOGI(TAG, "Bottom ToF initialized and ranging");
            }
            if (g_tof_front.isInitialized()) {
                g_tof_front.startRanging();
                stampfly::StampFlyState::getInstance().setFrontToFAvailable(true);
                ESP_LOGI(TAG, "Front ToF initialized and ranging");
            } else {
                stampfly::StampFlyState::getInstance().setFrontToFAvailable(false);
                ESP_LOGI(TAG, "Front ToF not available (optional sensor)");
            }
        }
    }

    // Optical Flow (PMW3901) - SPI (uses constructor with default config)
    {
        try {
            auto cfg = stampfly::PMW3901::Config::defaultStampFly();
            g_optflow = new stampfly::PMW3901(cfg);
            ESP_LOGI(TAG, "Optical Flow initialized");
        } catch (const stampfly::PMW3901Exception& e) {
            ESP_LOGW(TAG, "Optical Flow init failed: %s", e.what());
        }
    }

    // Power Monitor (INA3221) - I2C
    {
        stampfly::PowerMonitor::Config cfg;
        cfg.i2c_bus = g_i2c_bus;
        cfg.i2c_addr = stampfly::INA3221_I2C_ADDR_GND;
        cfg.battery_channel = 1;  // Battery is connected to CH1
        cfg.shunt_resistor_ohm = 0.1f;

        ret = g_power.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Power Monitor init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Power Monitor initialized");
        }
    }

    return ESP_OK;
}

static esp_err_t initActuators()
{
    ESP_LOGI(TAG, "Initializing actuators...");
    esp_err_t ret;

    // Motor Driver
    {
        stampfly::MotorDriver::Config cfg;
        cfg.gpio[stampfly::MotorDriver::MOTOR_FR] = GPIO_MOTOR_M1;
        cfg.gpio[stampfly::MotorDriver::MOTOR_RR] = GPIO_MOTOR_M2;
        cfg.gpio[stampfly::MotorDriver::MOTOR_RL] = GPIO_MOTOR_M3;
        cfg.gpio[stampfly::MotorDriver::MOTOR_FL] = GPIO_MOTOR_M4;
        cfg.pwm_freq_hz = 150000;  // 150kHz
        cfg.pwm_resolution_bits = 8;

        ret = g_motor.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor Driver init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Motor Driver initialized");
        g_motor_ptr = &g_motor;  // Set pointer for CLI access
    }

    // LED
    {
        stampfly::LED::Config cfg;
        cfg.gpio = GPIO_LED;
        cfg.num_leds = 1;

        ret = g_led.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "LED init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "LED initialized");
            g_led_ptr = &g_led;  // Set pointer for CLI access
        }
    }

    // Buzzer
    {
        stampfly::Buzzer::Config cfg;
        cfg.gpio = GPIO_BUZZER;
        cfg.ledc_channel = 4;
        cfg.ledc_timer = 1;

        ret = g_buzzer.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Buzzer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Buzzer initialized");
        }
    }

    // Button
    {
        stampfly::Button::Config cfg;
        cfg.gpio = GPIO_BUTTON;
        cfg.debounce_ms = 50;

        ret = g_button.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Button init failed: %s", esp_err_to_name(ret));
        } else {
            g_button.setCallback(onButtonEvent);
            ESP_LOGI(TAG, "Button initialized");
        }
    }

    return ESP_OK;
}

static esp_err_t initEstimators()
{
    ESP_LOGI(TAG, "Initializing estimators...");

    // Initialize IMU filters
    for (int i = 0; i < 3; i++) {
        g_accel_lpf[i].init(400.0f, 50.0f);   // 400Hz sampling, 50Hz cutoff
        g_gyro_lpf[i].init(400.0f, 100.0f);   // 400Hz sampling, 100Hz cutoff
    }

    // Initialize magnetometer calibrator and load from NVS
    g_mag_calibrator = &g_mag_cal;  // Set global pointer for CLI access
    if (g_mag_cal.loadFromNVS() == ESP_OK) {
        ESP_LOGI(TAG, "Magnetometer calibration loaded from NVS");
    } else {
        ESP_LOGW(TAG, "No magnetometer calibration found in NVS");
    }

    // ESKF (15-state Error-State Kalman Filter)
    {
        auto& state = stampfly::StampFlyState::getInstance();
        auto cfg = stampfly::ESKF::Config::defaultConfig();
        // 地磁気はキャリブレーション済みの場合のみ有効化
        cfg.mag_enabled = g_mag_cal.isCalibrated();
        ESP_LOGI(TAG, "ESKF mag_enabled: %s", cfg.mag_enabled ? "true" : "false");
        esp_err_t ret = g_eskf.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ESKF init failed: %s", esp_err_to_name(ret));
            state.setESKFInitialized(false);
        } else {
            ESP_LOGI(TAG, "ESKF initialized (predict at 400Hz)");
            state.setESKFInitialized(true);

            // ジャイロバイアスキャリブレーション（静止状態で実行）
            if (g_imu.isInitialized()) {
                ESP_LOGI(TAG, "Calibrating gyro bias (keep device still)...");
                constexpr int CALIB_SAMPLES = 200;  // 200サンプル @ 400Hz = 0.5秒
                float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
                int valid_samples = 0;

                for (int i = 0; i < CALIB_SAMPLES; i++) {
                    stampfly::AccelData accel;
                    stampfly::GyroData gyro;
                    if (g_imu.readSensorData(accel, gyro) == ESP_OK) {
                        // BMI270座標系 → 機体座標系(NED) 変換
                        float gyro_body_x = gyro.y;     // Roll rate
                        float gyro_body_y = gyro.x;     // Pitch rate
                        float gyro_body_z = -gyro.z;    // Yaw rate

                        gyro_sum_x += gyro_body_x;
                        gyro_sum_y += gyro_body_y;
                        gyro_sum_z += gyro_body_z;
                        valid_samples++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(2));  // ~500Hz
                }

                if (valid_samples > 0) {
                    stampfly::math::Vector3 gyro_bias(
                        gyro_sum_x / valid_samples,
                        gyro_sum_y / valid_samples,
                        gyro_sum_z / valid_samples
                    );
                    g_eskf.setGyroBias(gyro_bias);
                    g_initial_gyro_bias = gyro_bias;  // binlog reset後に復元するため保存
                    ESP_LOGI(TAG, "Gyro bias set: [%.5f, %.5f, %.5f] rad/s",
                             gyro_bias.x, gyro_bias.y, gyro_bias.z);
                }
            }

            // 地磁気リファレンス取得（Magキャリブレーション済みの場合）
            if (g_mag_cal.isCalibrated() && g_mag.isInitialized()) {
                ESP_LOGI(TAG, "Acquiring mag reference (keep device still)...");
                constexpr int MAG_CALIB_SAMPLES = 100;  // 100サンプル @ 100Hz = 1秒
                stampfly::math::Vector3 mag_sum = stampfly::math::Vector3::zero();
                int mag_valid_samples = 0;

                for (int i = 0; i < MAG_CALIB_SAMPLES; i++) {
                    stampfly::MagData raw_mag;
                    if (g_mag.read(raw_mag) == ESP_OK) {
                        // センサ座標系 → 機体座標系変換
                        float mag_body_x = raw_mag.y;
                        float mag_body_y = raw_mag.x;
                        float mag_body_z = -raw_mag.z;
                        // キャリブレーション適用
                        float cal_x, cal_y, cal_z;
                        g_mag_cal.applyCalibration(mag_body_x, mag_body_y, mag_body_z, cal_x, cal_y, cal_z);
                        mag_sum.x += cal_x;
                        mag_sum.y += cal_y;
                        mag_sum.z += cal_z;
                        mag_valid_samples++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));  // ~100Hz
                }

                if (mag_valid_samples > 0) {
                    stampfly::math::Vector3 mag_ref(
                        mag_sum.x / mag_valid_samples,
                        mag_sum.y / mag_valid_samples,
                        mag_sum.z / mag_valid_samples
                    );
                    g_eskf.setMagReference(mag_ref);
                    g_mag_ref_set = true;
                    ESP_LOGI(TAG, "Mag reference set: [%.1f, %.1f, %.1f] uT",
                             mag_ref.x, mag_ref.y, mag_ref.z);
                }
            }
        }
    }

    // Simple Attitude Estimator (backup/complementary)
    {
        stampfly::AttitudeEstimator::Config cfg;
        cfg.gyro_weight = 0.98f;
        cfg.mag_declination = 0.0f;  // Adjust for local declination
        esp_err_t ret = g_attitude_est.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "AttitudeEstimator init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "AttitudeEstimator initialized");
        }
    }

    // Simple Altitude Estimator
    {
        stampfly::AltitudeEstimator::Config cfg;
        cfg.process_noise_alt = 0.01f;
        cfg.process_noise_vel = 0.1f;
        cfg.measurement_noise_baro = 1.0f;
        cfg.measurement_noise_tof = 0.05f;
        esp_err_t ret = g_altitude_est.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "AltitudeEstimator init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "AltitudeEstimator initialized");
        }
    }

    return ESP_OK;
}

static esp_err_t initCommunication()
{
    ESP_LOGI(TAG, "Initializing communication...");

    // ESP-NOW Controller Communication
    {
        stampfly::ControllerComm::Config cfg;
        cfg.wifi_channel = 1;
        cfg.timeout_ms = 500;

        esp_err_t ret = g_comm.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ControllerComm init failed: %s", esp_err_to_name(ret));
            return ret;
        }

        g_comm.setControlCallback(onControlPacket);

        // Load pairing from NVS
        if (g_comm.loadPairingFromNVS() == ESP_OK && g_comm.isPaired()) {
            ESP_LOGI(TAG, "Loaded pairing from NVS");
            stampfly::StampFlyState::getInstance().setPairingState(stampfly::PairingState::PAIRED);
        }

        g_comm.start();

        // Set global pointer for CLI access
        g_comm_ptr = &g_comm;

        ESP_LOGI(TAG, "ControllerComm initialized");
    }

    return ESP_OK;
}

static esp_err_t initCLI()
{
    ESP_LOGI(TAG, "Initializing CLI...");

    esp_err_t ret = g_cli.init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CLI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_cli.registerDefaultCommands();

    // binlog開始時にmag_refを設定するコールバックを登録 (後方互換性のため維持)
    g_cli.setBinlogStartCallback(onBinlogStart);

    ESP_LOGI(TAG, "CLI initialized");
    return ESP_OK;
}

static esp_err_t initLogger()
{
    ESP_LOGI(TAG, "Initializing Logger...");

    // 400Hz logging (matches ESKF rate)
    esp_err_t ret = g_logger.init(400);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Logger init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set start callback (ESKF reset + mag_ref setting)
    g_logger.setStartCallback(onBinlogStart);

    // Set global pointer for CLI access
    g_logger_ptr = &g_logger;

    ESP_LOGI(TAG, "Logger initialized at 400Hz");
    return ESP_OK;
}

static esp_err_t initTelemetry()
{
    ESP_LOGI(TAG, "Initializing Telemetry...");

    auto& telemetry = stampfly::Telemetry::getInstance();
    stampfly::Telemetry::Config cfg;
    cfg.port = 80;
    cfg.rate_hz = 50;

    esp_err_t ret = telemetry.init(cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Telemetry init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Telemetry initialized - Connect to WiFi 'StampFly', open http://192.168.4.1");
    return ESP_OK;
}

static void startTasks()
{
    ESP_LOGI(TAG, "Starting FreeRTOS tasks...");

    // Peripheral tasks (Core 0)
    xTaskCreatePinnedToCore(LEDTask, "LEDTask", STACK_SIZE_LED, nullptr,
                            PRIORITY_LED_TASK, &g_led_task_handle, 0);

    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", STACK_SIZE_BUTTON, nullptr,
                            PRIORITY_BUTTON_TASK, &g_button_task_handle, 0);

    xTaskCreatePinnedToCore(PowerTask, "PowerTask", STACK_SIZE_POWER, nullptr,
                            PRIORITY_POWER_TASK, &g_power_task_handle, 0);

    // Initialize IMU timer and semaphores for precise 400Hz timing
    g_imu_semaphore = xSemaphoreCreateBinary();
    if (g_imu_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create IMU semaphore");
    }

    g_control_semaphore = xSemaphoreCreateBinary();
    if (g_control_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create control semaphore");
    }

    esp_timer_create_args_t imu_timer_args = {
        .callback = imu_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,  // Run in esp_timer task context
        .name = "imu_timer",
        .skip_unhandled_events = true,      // Skip if semaphore already given
    };
    esp_err_t ret = esp_timer_create(&imu_timer_args, &g_imu_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create IMU timer: %s", esp_err_to_name(ret));
    } else {
        // Start periodic timer: 2500μs = 2.5ms = 400Hz
        ret = esp_timer_start_periodic(g_imu_timer, 2500);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start IMU timer: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "IMU timer started: 2.5ms period (400Hz)");
        }
    }

    // Sensor tasks (Core 1)
    xTaskCreatePinnedToCore(IMUTask, "IMUTask", STACK_SIZE_IMU, nullptr,
                            PRIORITY_IMU_TASK, &g_imu_task_handle, 1);

    // Control task (Core 1) - runs at 400Hz, synchronized with IMU via semaphore
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", STACK_SIZE_CONTROL, nullptr,
                            PRIORITY_CONTROL_TASK, &g_control_task_handle, 1);

    // I2C sensor tasks (Core 0) - moved from Core 1 to avoid contention with 400Hz IMU
    xTaskCreatePinnedToCore(MagTask, "MagTask", STACK_SIZE_MAG, nullptr,
                            PRIORITY_MAG_TASK, &g_mag_task_handle, 0);

    xTaskCreatePinnedToCore(BaroTask, "BaroTask", STACK_SIZE_BARO, nullptr,
                            PRIORITY_BARO_TASK, &g_baro_task_handle, 0);

    xTaskCreatePinnedToCore(ToFTask, "ToFTask", STACK_SIZE_TOF, nullptr,
                            PRIORITY_TOF_TASK, &g_tof_task_handle, 0);

    xTaskCreatePinnedToCore(OptFlowTask, "OptFlowTask", STACK_SIZE_OPTFLOW, nullptr,
                            PRIORITY_OPTFLOW_TASK, &g_optflow_task_handle, 1);

    // Communication task (Core 0)
    xTaskCreatePinnedToCore(CommTask, "CommTask", STACK_SIZE_COMM, nullptr,
                            PRIORITY_COMM_TASK, &g_comm_task_handle, 0);

    // CLI task (Core 0)
    xTaskCreatePinnedToCore(CLITask, "CLITask", STACK_SIZE_CLI, nullptr,
                            PRIORITY_CLI_TASK, &g_cli_task_handle, 0);

    // Telemetry task (Core 0) - WebSocket broadcast at 50Hz
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", STACK_SIZE_TELEMETRY, nullptr,
                            PRIORITY_TELEMETRY_TASK, &g_telemetry_task_handle, 0);

    ESP_LOGI(TAG, "All tasks started");
}

// =============================================================================
// Main Entry Point
// =============================================================================

extern "C" void app_main(void)
{
    // Delay to allow USB to connect for debug output
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Direct printf for early debug (before ESP_LOG may be configured)
    printf("\n\n*** StampFly Boot Start ***\n\n");
    fflush(stdout);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  StampFly RTOS Skeleton");
    ESP_LOGI(TAG, "  ESP-IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");

    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize network interface and event loop (required for WiFi/ESP-NOW)
    ESP_LOGI(TAG, "Initializing network interface...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Network interface initialized");

    // Initialize state manager
    ESP_LOGI(TAG, "Initializing state manager...");
    auto& state = stampfly::StampFlyState::getInstance();
    ESP_ERROR_CHECK(state.init());

    // Initialize system manager
    ESP_LOGI(TAG, "Initializing system manager...");
    auto& sys_mgr = stampfly::SystemManager::getInstance();
    stampfly::SystemManager::Config sys_cfg;
    sys_cfg.init_timeout_ms = 5000;
    sys_cfg.calib_timeout_ms = 10000;
    ESP_ERROR_CHECK(sys_mgr.init(sys_cfg));

    // Initialize I2C bus first (required for I2C sensors)
    ESP_LOGI(TAG, "Initializing I2C...");
    initI2C();

    // Initialize actuators first (for buzzer feedback)
    ESP_LOGI(TAG, "Initializing actuators...");
    initActuators();

    // Play startup tone early to indicate boot progress
    ESP_LOGI(TAG, "Playing startup tone...");
    g_buzzer.startTone();
    g_led.showInit();

    // Initialize sensors (may fail partially, that's OK)
    ESP_LOGI(TAG, "Initializing sensors...");
    initSensors();

    // Initialize estimators
    ESP_LOGI(TAG, "Initializing estimators...");
    initEstimators();

    // Initialize communication (ESP-NOW)
    ESP_LOGI(TAG, "Initializing communication...");
    initCommunication();

    // Initialize CLI
    ESP_LOGI(TAG, "Initializing CLI...");
    initCLI();

    // Initialize Logger (400Hz binary log)
    ESP_LOGI(TAG, "Initializing Logger...");
    initLogger();

    // Initialize Telemetry (WebSocket server)
    ESP_LOGI(TAG, "Initializing Telemetry...");
    initTelemetry();

    // Start all tasks
    ESP_LOGI(TAG, "Starting tasks...");
    startTasks();

    // Transition to IDLE state after initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
    state.setFlightState(stampfly::FlightState::IDLE);

    // Wait for sensors to stabilize (ESKF is NOT running during this period)
    // g_eskf_ready = false なので IMUTask は ESKF 処理をスキップしている
    ESP_LOGI(TAG, "Waiting for sensors to stabilize (ESKF paused)...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Initialize ESKF with fresh state and stable sensor data
    // センサーが安定した状態でESKFをリセット・キャリブレーション
    if (g_eskf.isInitialized()) {
        g_eskf.reset();
        g_eskf.setGyroBias(g_initial_gyro_bias);
        setMagReferenceFromBuffer();

        // ESKF処理を開始
        g_eskf_ready = true;
        ESP_LOGI(TAG, "ESKF initialized with stable sensor data, ready to run");
    }

    // ESKF ready notification - 3 short beeps + green LED
    ESP_LOGI(TAG, "ESKF ready - playing notification...");
    g_buzzer.beep();
    vTaskDelay(pdMS_TO_TICKS(150));
    g_buzzer.beep();
    vTaskDelay(pdMS_TO_TICKS(150));
    g_buzzer.beep();
    g_led.setPattern(stampfly::LED::Pattern::SOLID, 0x00FF00);  // Green = ready

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "StampFly initialized successfully!");
    ESP_LOGI(TAG, "ESKF ready - you can start logging now");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Suppressing ESP_LOG output for CLI. Use 'loglevel' to re-enable.");

    // Suppress ESP_LOG after initialization to keep CLI clean
    esp_log_level_set("*", ESP_LOG_NONE);

    // Main loop - just monitor system health
    while (true) {
        // Heap monitoring disabled by default (ESP_LOG suppressed)
        // Use 'loglevel info' CLI command to re-enable if needed

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
