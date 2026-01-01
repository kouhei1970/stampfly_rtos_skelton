/**
 * @file sensor_fusion.cpp
 * @brief センサーフュージョン実装
 */

#include "sensor_fusion.hpp"
#include <cmath>

namespace sf {

bool SensorFusion::init() {
    Config config;
    return init(config);
}

bool SensorFusion::init(const Config& config) {
    config_ = config;

    // ESKF初期化（デフォルト設定）
    auto eskf_config = stampfly::ESKF::Config::defaultConfig();
    if (eskf_.init(eskf_config) != ESP_OK) {
        return false;
    }

    initialized_ = true;
    diverged_ = false;
    return true;
}

bool SensorFusion::predictIMU(const stampfly::math::Vector3& accel_body,
                               const stampfly::math::Vector3& gyro_body,
                               float dt) {
    if (!initialized_) {
        return false;
    }

    // 入力値の事前チェック
    bool input_valid = std::isfinite(accel_body.x) && std::isfinite(accel_body.y) &&
                       std::isfinite(accel_body.z) && std::isfinite(gyro_body.x) &&
                       std::isfinite(gyro_body.y) && std::isfinite(gyro_body.z);

    if (!input_valid) {
        return false;
    }

    // ESKF予測ステップ
    eskf_.predict(accel_body, gyro_body, dt);

    // 加速度計による姿勢補正
    eskf_.updateAccelAttitude(accel_body);

    // 発散チェック
    auto state = eskf_.getState();
    diverged_ = checkDivergence(state);

    return !diverged_;
}

void SensorFusion::updateOpticalFlow(int16_t dx, int16_t dy, uint8_t squal,
                                      float distance, float dt,
                                      float gyro_x, float gyro_y) {
    if (!initialized_ || diverged_) {
        return;
    }

    // センサー無効時はスキップ
    if (!config_.use_optical_flow) {
        return;
    }

    // 品質チェック（squal >= 0x19 で有効）
    if (squal < 0x19) {
        return;
    }

    // 距離の有効範囲チェック
    if (distance < 0.02f || distance > 4.0f) {
        return;
    }

    // ESKF更新
    eskf_.updateFlowRaw(dx, dy, distance, dt, gyro_x, gyro_y);
}

void SensorFusion::updateBarometer(float relative_altitude) {
    if (!initialized_ || diverged_) {
        return;
    }

    // センサー無効時はスキップ
    if (!config_.use_barometer) {
        return;
    }

    eskf_.updateBaro(relative_altitude);
}

void SensorFusion::updateToF(float distance) {
    if (!initialized_ || diverged_) {
        return;
    }

    // センサー無効時はスキップ
    if (!config_.use_tof) {
        return;
    }

    // 距離の有効範囲チェック
    if (distance < 0.01f || distance > 4.0f) {
        return;
    }

    eskf_.updateToF(distance);
}

void SensorFusion::updateMagnetometer(const stampfly::math::Vector3& mag_body) {
    if (!initialized_ || diverged_) {
        return;
    }

    // センサー無効時はスキップ
    if (!config_.use_magnetometer) {
        return;
    }

    eskf_.updateMag(mag_body);
}

SensorFusion::State SensorFusion::getState() const {
    State result;

    if (!initialized_) {
        return result;
    }

    auto eskf_state = eskf_.getState();
    result.roll = eskf_state.roll;
    result.pitch = eskf_state.pitch;
    result.yaw = eskf_state.yaw;
    result.position = eskf_state.position;
    result.velocity = eskf_state.velocity;
    result.gyro_bias = eskf_state.gyro_bias;
    result.accel_bias = eskf_state.accel_bias;

    return result;
}

void SensorFusion::reset() {
    if (!initialized_) {
        return;
    }

    eskf_.reset();
    diverged_ = false;
}

void SensorFusion::setGyroBias(const stampfly::math::Vector3& bias) {
    if (!initialized_) {
        return;
    }

    eskf_.setGyroBias(bias);
}

void SensorFusion::setMagReference(const stampfly::math::Vector3& ref) {
    if (!initialized_) {
        return;
    }

    eskf_.setMagReference(ref);
}

bool SensorFusion::checkDivergence(const stampfly::ESKF::State& state) {
    // 姿勢の有効性チェック
    if (!std::isfinite(state.roll) || !std::isfinite(state.pitch)) {
        return true;
    }

    // 位置の発散検出
    if (std::abs(state.position.x) > config_.max_position ||
        std::abs(state.position.y) > config_.max_position ||
        std::abs(state.position.z) > config_.max_position) {
        return true;
    }

    // 速度の発散検出
    if (std::abs(state.velocity.x) > config_.max_velocity ||
        std::abs(state.velocity.y) > config_.max_velocity ||
        std::abs(state.velocity.z) > config_.max_velocity) {
        return true;
    }

    return false;
}

} // namespace sf
