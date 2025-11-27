/**
 * @file eskf.hpp
 * @brief Error-State Kalman Filter (ESKF) for Attitude, Position, and Velocity Estimation
 *
 * 15状態ESKF統合推定器:
 * - 位置 (3次元)
 * - 速度 (3次元)
 * - 姿勢 (クォータニオン→エラー状態は3次元回転ベクトル)
 * - ジャイロバイアス (3次元)
 * - 加速度バイアス (3次元)
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "stampfly_math.hpp"

namespace stampfly {

using namespace math;

/**
 * @brief ESKF 統合推定器
 */
class ESKF {
public:
    struct Config {
        // プロセスノイズ
        float gyro_noise;           // ジャイロノイズ [rad/s/√Hz]
        float accel_noise;          // 加速度ノイズ [m/s²/√Hz]
        float gyro_bias_noise;      // ジャイロバイアスランダムウォーク
        float accel_bias_noise;     // 加速度バイアスランダムウォーク

        // 観測ノイズ
        float baro_noise;           // 気圧高度ノイズ [m]
        float tof_noise;            // ToFノイズ [m]
        float mag_noise;            // 地磁気ノイズ [uT]
        float flow_noise;           // オプティカルフローノイズ

        // 初期共分散
        float init_pos_std;
        float init_vel_std;
        float init_att_std;
        float init_gyro_bias_std;
        float init_accel_bias_std;

        // 地磁気参照ベクトル (NED)
        Vector3 mag_ref;

        // 重力加速度
        float gravity;

        // デフォルト設定
        static Config defaultConfig() {
            Config cfg;
            cfg.gyro_noise = 0.01f;
            cfg.accel_noise = 0.1f;
            cfg.gyro_bias_noise = 0.0001f;
            cfg.accel_bias_noise = 0.001f;
            cfg.baro_noise = 1.0f;
            cfg.tof_noise = 0.05f;
            cfg.mag_noise = 5.0f;
            cfg.flow_noise = 0.5f;
            cfg.init_pos_std = 1.0f;
            cfg.init_vel_std = 0.5f;
            cfg.init_att_std = 0.1f;
            cfg.init_gyro_bias_std = 0.01f;
            cfg.init_accel_bias_std = 0.1f;
            cfg.mag_ref = Vector3(20.0f, 0.0f, 40.0f);  // 日本近辺の概算
            cfg.gravity = 9.81f;
            return cfg;
        }
    };

    struct State {
        Vector3 position;           // 位置 [m] (NED)
        Vector3 velocity;           // 速度 [m/s]
        Quaternion orientation;     // 姿勢
        Vector3 gyro_bias;          // ジャイロバイアス [rad/s]
        Vector3 accel_bias;         // 加速度バイアス [m/s²]

        // オイラー角 (便利用)
        float roll;                 // [rad]
        float pitch;                // [rad]
        float yaw;                  // [rad]
    };

    ESKF() = default;

    /**
     * @brief 初期化
     */
    esp_err_t init(const Config& config);

    /**
     * @brief 予測ステップ (IMU入力)
     * @param accel 加速度 [m/s²] (ボディ座標系)
     * @param gyro 角速度 [rad/s] (ボディ座標系)
     * @param dt 時間刻み [s]
     */
    void predict(const Vector3& accel, const Vector3& gyro, float dt);

    /**
     * @brief 気圧高度更新
     * @param altitude 高度 [m]
     */
    void updateBaro(float altitude);

    /**
     * @brief ToF更新 (姿勢補正込み)
     * @param distance 距離 [m]
     */
    void updateToF(float distance);

    /**
     * @brief 地磁気更新 (ヨー補正)
     * @param mag 地磁気 [uT] (ボディ座標系)
     */
    void updateMag(const Vector3& mag);

    /**
     * @brief オプティカルフロー更新 (水平速度)
     * @param flow_x X方向フロー [rad/s]
     * @param flow_y Y方向フロー [rad/s]
     * @param height 高度 [m]
     */
    void updateFlow(float flow_x, float flow_y, float height);

    /**
     * @brief 現在の状態取得
     */
    State getState() const { return state_; }

    /**
     * @brief 状態リセット
     */
    void reset();

    /**
     * @brief 共分散行列取得 (デバッグ用)
     */
    const Matrix<15, 15>& getCovariance() const { return P_; }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;

    // 名目状態
    State state_;

    // エラー状態共分散行列 (15x15)
    // 状態順序: [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]
    Matrix<15, 15> P_;

    /**
     * @brief エラー状態を名目状態に注入
     * @param dx 15次元エラー状態
     */
    void injectErrorState(const Matrix<15, 1>& dx);

    /**
     * @brief EKF更新（汎用）
     */
    template<int M>
    void measurementUpdate(const Matrix<M, 1>& z,
                           const Matrix<M, 1>& h,
                           const Matrix<M, 15>& H,
                           const Matrix<M, M>& R);
};

/**
 * @brief 簡易姿勢推定器 (相補フィルタ)
 *
 * ESKF不要の場合の軽量版
 */
class AttitudeEstimator {
public:
    struct Config {
        float gyro_weight;          // ジャイロ重み (default: 0.98)
        float mag_declination;      // 地磁気偏角 [rad]
    };

    struct State {
        Quaternion orientation;
        float roll;                 // [rad]
        float pitch;                // [rad]
        float yaw;                  // [rad]
        Vector3 gyro_bias;
    };

    AttitudeEstimator() = default;

    esp_err_t init(const Config& config);
    void update(const Vector3& accel, const Vector3& gyro, float dt);
    void updateMag(const Vector3& mag);
    State getState() const { return state_; }
    void reset();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    State state_;
};

/**
 * @brief 簡易高度推定器 (カルマンフィルタ)
 */
class AltitudeEstimator {
public:
    struct Config {
        float process_noise_alt;
        float process_noise_vel;
        float measurement_noise_baro;
        float measurement_noise_tof;
    };

    struct State {
        float altitude;             // [m]
        float velocity;             // [m/s]
    };

    AltitudeEstimator() = default;

    esp_err_t init(const Config& config);
    void predict(float accel_z, float dt);
    void updateBaro(float altitude);
    void updateToF(float distance, float pitch, float roll);
    State getState() const { return state_; }
    void reset();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    State state_;
    float P_[2][2] = {{1, 0}, {0, 1}};
};

/**
 * @brief 簡易速度推定器 (オプティカルフロー)
 */
class VelocityEstimator {
public:
    struct Config {
        float flow_scale;           // フロースケール
    };

    struct State {
        float vx;                   // [m/s] (ボディ座標系)
        float vy;                   // [m/s] (ボディ座標系)
    };

    VelocityEstimator() = default;

    esp_err_t init(const Config& config);
    void updateFlow(float flow_x, float flow_y, float height, float gyro_x, float gyro_y);
    State getState() const { return state_; }
    void reset();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    State state_;
};

}  // namespace stampfly
