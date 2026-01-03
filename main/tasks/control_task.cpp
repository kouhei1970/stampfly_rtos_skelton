/**
 * @file control_task.cpp
 * @brief 制御タスク (400Hz) - 角速度制御（Rate Control）
 *
 * コントローラ入力から目標角速度を計算し、PID制御でモーター出力を決定
 */

#include "tasks_common.hpp"
#include "../rate_controller.hpp"

static const char* TAG = "ControlTask";

using namespace config;
using namespace globals;

// Debug: Yaw alert counter (1秒 = 400カウント @ 400Hz)
volatile int g_yaw_alert_counter = 0;

// 衝撃検出用カウンタ
static int g_impact_count = 0;

// =============================================================================
// Rate Controller Implementation
// =============================================================================

void RateController::init() {
    // デフォルト値で初期化
    roll_rate_max = rate_control::ROLL_RATE_MAX;
    pitch_rate_max = rate_control::PITCH_RATE_MAX;
    yaw_rate_max = rate_control::YAW_RATE_MAX;

    // Roll PID
    stampfly::PIDConfig roll_cfg;
    roll_cfg.Kp = rate_control::ROLL_RATE_KP;
    roll_cfg.Ti = rate_control::ROLL_RATE_TI;
    roll_cfg.Td = rate_control::ROLL_RATE_TD;
    roll_cfg.eta = rate_control::PID_ETA;
    roll_cfg.output_min = -rate_control::OUTPUT_LIMIT;
    roll_cfg.output_max = rate_control::OUTPUT_LIMIT;
    roll_cfg.derivative_on_measurement = true;
    roll_pid.init(roll_cfg);

    // Pitch PID
    stampfly::PIDConfig pitch_cfg;
    pitch_cfg.Kp = rate_control::PITCH_RATE_KP;
    pitch_cfg.Ti = rate_control::PITCH_RATE_TI;
    pitch_cfg.Td = rate_control::PITCH_RATE_TD;
    pitch_cfg.eta = rate_control::PID_ETA;
    pitch_cfg.output_min = -rate_control::OUTPUT_LIMIT;
    pitch_cfg.output_max = rate_control::OUTPUT_LIMIT;
    pitch_cfg.derivative_on_measurement = true;
    pitch_pid.init(pitch_cfg);

    // Yaw PID
    stampfly::PIDConfig yaw_cfg;
    yaw_cfg.Kp = rate_control::YAW_RATE_KP;
    yaw_cfg.Ti = rate_control::YAW_RATE_TI;
    yaw_cfg.Td = rate_control::YAW_RATE_TD;
    yaw_cfg.eta = rate_control::PID_ETA;
    yaw_cfg.output_min = -rate_control::OUTPUT_LIMIT;
    yaw_cfg.output_max = rate_control::OUTPUT_LIMIT;
    yaw_cfg.derivative_on_measurement = true;
    yaw_pid.init(yaw_cfg);

    initialized = true;
    ESP_LOGI(TAG, "RateController initialized");
    ESP_LOGI(TAG, "  Sensitivity: R=%.1f P=%.1f Y=%.1f [rad/s]",
             roll_rate_max, pitch_rate_max, yaw_rate_max);
    ESP_LOGI(TAG, "  Roll  PID: Kp=%.3f Ti=%.3f Td=%.4f",
             roll_cfg.Kp, roll_cfg.Ti, roll_cfg.Td);
    ESP_LOGI(TAG, "  Pitch PID: Kp=%.3f Ti=%.3f Td=%.4f",
             pitch_cfg.Kp, pitch_cfg.Ti, pitch_cfg.Td);
    ESP_LOGI(TAG, "  Yaw   PID: Kp=%.3f Ti=%.3f Td=%.4f",
             yaw_cfg.Kp, yaw_cfg.Ti, yaw_cfg.Td);
}

void RateController::reset() {
    roll_pid.reset();
    pitch_pid.reset();
    yaw_pid.reset();
}

// グローバルレートコントローラ（CLIからアクセス可能）
RateController g_rate_controller;

// CLIからアクセスするためのポインタ
RateController* g_rate_controller_ptr = &g_rate_controller;

/**
 * @brief Control Task - 400Hz (2.5ms period)
 *
 * 角速度制御ループ:
 * 1. コントローラ入力から目標角速度を計算
 * 2. IMUから現在の角速度を取得（バイアス補正済み）
 * 3. PID制御で制御出力を計算
 * 4. モーターミキサーで各モーター出力を決定
 *
 * Motor Layout (X-quad configuration, viewed from above)
 *
 *               Front
 *          FL (M4)   FR (M1)
 *             ╲   ▲   ╱
 *              ╲  │  ╱
 *               ╲ │ ╱
 *                ╲│╱
 *                 ╳         ← Center of drone
 *                ╱│╲
 *               ╱ │ ╲
 *              ╱  │  ╲
 *             ╱   │   ╲
 *          RL (M3)    RR (M2)
 *                Rear
 *
 * Motor rotation:
 *   M1 (FR): CCW (Counter-Clockwise)
 *   M2 (RR): CW  (Clockwise)
 *   M3 (RL): CCW (Counter-Clockwise)
 *   M4 (FL): CW  (Clockwise)
 */
void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started (400Hz via semaphore)");

    auto& state = stampfly::StampFlyState::getInstance();

    // レートコントローラ初期化
    g_rate_controller.init();

    // 前回のフライト状態（ARMED遷移時にPIDリセット用）
    stampfly::FlightState prev_flight_state = stampfly::FlightState::INIT;

    while (true) {
        // Wait for control semaphore (given by IMU task after ESKF update)
        if (xSemaphoreTake(g_control_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Get current flight state
        stampfly::FlightState flight_state = state.getFlightState();

        // ARMED遷移時にPIDをリセット（積分項クリア）
        if (flight_state == stampfly::FlightState::ARMED &&
            prev_flight_state != stampfly::FlightState::ARMED) {
            g_rate_controller.reset();
            ESP_LOGI(TAG, "PID reset on ARM");
        }
        prev_flight_state = flight_state;

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

        // =====================================================================
        // 1. コントローラ入力取得
        // =====================================================================
        // throttle: 0.0 ~ 1.0
        // roll, pitch, yaw: -1.0 ~ +1.0
        float throttle, roll_cmd, pitch_cmd, yaw_cmd;
        state.getControlInput(throttle, roll_cmd, pitch_cmd, yaw_cmd);

        // =====================================================================
        // 2. 目標角速度計算
        // =====================================================================
        // スティック入力 × 感度 = 目標角速度 [rad/s]
        float roll_rate_target = roll_cmd * g_rate_controller.roll_rate_max;
        float pitch_rate_target = pitch_cmd * g_rate_controller.pitch_rate_max;
        float yaw_rate_target = yaw_cmd * g_rate_controller.yaw_rate_max;

        // =====================================================================
        // 3. 現在の角速度取得（生データ - バイアス推定が不安定な場合用）
        // =====================================================================
        stampfly::Vec3 accel, gyro;
        state.getIMUData(accel, gyro);  // TODO: バイアス推定安定後は getIMUCorrected() に戻す

        // =====================================================================
        // 衝撃検出 - 加速度が閾値を超えたら自動Disarm
        // =====================================================================
        float accel_magnitude = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        if (accel_magnitude > safety::IMPACT_THRESHOLD_MS2) {
            g_impact_count++;
            if (g_impact_count >= safety::IMPACT_COUNT_THRESHOLD) {
                // 衝撃検出 → 自動Disarm
                ESP_LOGE(TAG, "IMPACT DETECTED! accel=%.1f m/s^2 (%.1fG) - Auto DISARM",
                         accel_magnitude, accel_magnitude / 9.81f);

                // Disarm処理
                if (state.requestDisarm()) {
                    g_motor.disarm();
                    g_buzzer.errorTone();
                    stampfly::LEDManager::getInstance().requestChannel(
                        stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::CRITICAL_ERROR,
                        stampfly::LEDPattern::BLINK_FAST, 0xFF0000, 5000);  // 赤点滅5秒
                    ESP_LOGE(TAG, "Motors DISARMED due to impact");
                }
                g_impact_count = 0;
                continue;  // このサイクルはスキップ
            }
        } else {
            g_impact_count = 0;  // 閾値以下ならリセット
        }

        float roll_rate_current = gyro.x;   // [rad/s]
        float pitch_rate_current = gyro.y;  // [rad/s]
        float yaw_rate_current = gyro.z;    // [rad/s]

        // デバッグ: ヨー関連の急変検出 → LED表示（1秒間維持）
        static float prev_yaw_cmd = 0.0f;
        static float prev_yaw_gyro = 0.0f;
        float yaw_cmd_diff = std::abs(yaw_cmd - prev_yaw_cmd);
        float yaw_gyro_diff = std::abs(yaw_rate_current - prev_yaw_gyro);
        auto& led_mgr = stampfly::LEDManager::getInstance();
        if (yaw_cmd_diff > 0.3f) {
            // 指令が急変 → 黄色点滅（1秒間）- DEBUG_ALERT優先度、タイムアウト付き
            led_mgr.requestChannel(stampfly::LEDChannel::STATUS,
                stampfly::LEDPriority::DEBUG_ALERT,
                stampfly::LEDPattern::BLINK_FAST, 0xFFFF00, 1000);
            g_yaw_alert_counter = 400;
        }
        if (yaw_gyro_diff > 1.0f) {
            // ジャイロが急変 → 黄色点灯（1秒間）- DEBUG_ALERT優先度、タイムアウト付き
            led_mgr.requestChannel(stampfly::LEDChannel::STATUS,
                stampfly::LEDPriority::DEBUG_ALERT,
                stampfly::LEDPattern::SOLID, 0xFFFF00, 1000);
            g_yaw_alert_counter = 400;
        }
        if (g_yaw_alert_counter > 0) {
            g_yaw_alert_counter--;
        }
        prev_yaw_cmd = yaw_cmd;
        prev_yaw_gyro = yaw_rate_current;

        // =====================================================================
        // 4. PID制御
        // =====================================================================
        constexpr float dt = IMU_DT;  // 2.5ms

        float roll_out = g_rate_controller.roll_pid.update(
            roll_rate_target, roll_rate_current, dt);
        float pitch_out = g_rate_controller.pitch_pid.update(
            pitch_rate_target, pitch_rate_current, dt);
        float yaw_out = g_rate_controller.yaw_pid.update(
            yaw_rate_target, yaw_rate_current, dt);

        // =====================================================================
        // 5. モーターミキサー
        // =====================================================================
        // X-quad mixer: setMixerOutput handles the motor mixing
        // thrust: 0.0 ~ 1.0
        // roll/pitch/yaw: -1.0 ~ +1.0 (already limited by PID output limits)
        g_motor.setMixerOutput(throttle, roll_out, pitch_out, yaw_out);
    }
}
