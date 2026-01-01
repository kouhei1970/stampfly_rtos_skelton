/**
 * @file sensor_fusion.hpp
 * @brief センサーフュージョン - ESKFを用いた状態推定
 *
 * このコンポーネントはESKFの呼び出しパターンをカプセル化し、
 * 「センサーフュージョンとは何か」を学ぶための入り口を提供する。
 *
 * 設計原則:
 * - FreeRTOS非依存（algo_*レイヤー）
 * - ESKFを内包し、更新タイミングを管理
 * - 発散検出・自動リセット機能
 */

#pragma once

#include "eskf.hpp"
#include "stampfly_math.hpp"

namespace sf {

/**
 * @brief センサーフュージョンクラス
 *
 * 使用例:
 * @code
 * sf::SensorFusion fusion;
 * fusion.init();
 *
 * // 400Hzメインループ
 * fusion.predictIMU(accel_body, gyro_body, dt);
 *
 * // 各センサー更新（非同期）
 * if (flow_ready) fusion.updateOpticalFlow(...);
 * if (tof_ready)  fusion.updateToF(...);
 * @endcode
 */
class SensorFusion {
public:
    /**
     * @brief 設定構造体
     */
    struct Config {
        // センサー使用スイッチ（デフォルト: 全て有効）
        bool use_optical_flow = true;
        bool use_barometer = true;
        bool use_tof = true;
        bool use_magnetometer = true;

        // 発散検出閾値
        float max_position = 100.0f;   // [m]
        float max_velocity = 50.0f;    // [m/s]

        Config() = default;
    };

    /**
     * @brief 推定状態
     */
    struct State {
        float roll = 0.0f;           // [rad]
        float pitch = 0.0f;          // [rad]
        float yaw = 0.0f;            // [rad]
        stampfly::math::Vector3 position;   // [m] NED
        stampfly::math::Vector3 velocity;   // [m/s] NED
        stampfly::math::Vector3 gyro_bias;  // [rad/s]
        stampfly::math::Vector3 accel_bias; // [m/s²]
    };

    SensorFusion() = default;
    ~SensorFusion() = default;

    // コピー禁止
    SensorFusion(const SensorFusion&) = delete;
    SensorFusion& operator=(const SensorFusion&) = delete;

    /**
     * @brief 初期化（デフォルト設定）
     */
    bool init();

    /**
     * @brief 初期化（カスタム設定）
     */
    bool init(const Config& config);

    /**
     * @brief 初期化済みか
     */
    bool isInitialized() const { return initialized_; }

    // =========================================================================
    // IMU更新（400Hz メインループから呼び出し）
    // =========================================================================

    /**
     * @brief IMU予測ステップ + 加速度計姿勢補正
     * @param accel_body 加速度 [m/s²] 機体座標系
     * @param gyro_body 角速度 [rad/s] 機体座標系
     * @param dt 時間刻み [s]
     * @return 発散していない場合true
     */
    bool predictIMU(const stampfly::math::Vector3& accel_body,
                    const stampfly::math::Vector3& gyro_body,
                    float dt);

    // =========================================================================
    // 各センサー更新（非同期、データ準備時に呼び出し）
    // =========================================================================

    /**
     * @brief オプティカルフロー更新
     */
    void updateOpticalFlow(int16_t dx, int16_t dy, float distance,
                           float dt, float gyro_x, float gyro_y);

    /**
     * @brief 気圧高度更新
     */
    void updateBarometer(float relative_altitude);

    /**
     * @brief ToF距離更新
     */
    void updateToF(float distance);

    /**
     * @brief 磁力計更新
     */
    void updateMagnetometer(const stampfly::math::Vector3& mag_body);

    // =========================================================================
    // 状態取得
    // =========================================================================

    /**
     * @brief 現在の推定状態を取得
     */
    State getState() const;

    /**
     * @brief 発散しているか
     */
    bool isDiverged() const { return diverged_; }

    /**
     * @brief ESKFリセット
     */
    void reset();

    /**
     * @brief ジャイロバイアス設定
     */
    void setGyroBias(const stampfly::math::Vector3& bias);

    /**
     * @brief 磁気リファレンス設定
     */
    void setMagReference(const stampfly::math::Vector3& ref);

    /**
     * @brief 内部ESKFへの直接アクセス（上級者向け）
     */
    stampfly::ESKF& getESKF() { return eskf_; }
    const stampfly::ESKF& getESKF() const { return eskf_; }

private:
    bool initialized_ = false;
    bool diverged_ = false;
    Config config_;
    stampfly::ESKF eskf_;

    bool checkDivergence(const stampfly::ESKF::State& state);
};

} // namespace sf
