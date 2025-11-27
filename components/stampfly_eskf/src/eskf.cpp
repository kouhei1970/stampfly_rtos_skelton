/**
 * @file eskf.cpp
 * @brief Attitude/Position Estimator Implementation
 */
#include "eskf.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

static const char* TAG = "Estimator";

namespace stampfly {

// ============================================================================
// Vec3 Implementation
// ============================================================================

float Vec3::norm() const {
    return std::sqrt(x * x + y * y + z * z);
}

Vec3 Vec3::normalized() const {
    float n = norm();
    if (n < 1e-6f) return Vec3(0, 0, 0);
    return Vec3(x / n, y / n, z / n);
}

// ============================================================================
// Quaternion Implementation
// ============================================================================

Quat Quat::operator*(const Quat& q) const {
    return Quat(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

float Quat::norm() const {
    return std::sqrt(w * w + x * x + y * y + z * z);
}

void Quat::normalize() {
    float n = norm();
    if (n < 1e-6f) {
        w = 1.0f; x = y = z = 0.0f;
        return;
    }
    w /= n; x /= n; y /= n; z /= n;
}

Vec3 Quat::rotate(const Vec3& v) const {
    Quat p(0, v.x, v.y, v.z);
    Quat result = (*this) * p * conjugate();
    return Vec3(result.x, result.y, result.z);
}

void Quat::toEuler(float& roll, float& pitch, float& yaw) const {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (std::fabs(sinp) >= 1.0f) {
        pitch = std::copysign(M_PI / 2.0f, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

Quat Quat::fromEuler(float roll, float pitch, float yaw) {
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);

    return Quat(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );
}

Quat Quat::fromAxisAngle(const Vec3& axis, float angle) {
    Vec3 n = axis.normalized();
    float s = std::sin(angle * 0.5f);
    return Quat(std::cos(angle * 0.5f), n.x * s, n.y * s, n.z * s);
}

// ============================================================================
// AttitudeEstimator Implementation
// ============================================================================

AttitudeEstimator& AttitudeEstimator::getInstance() {
    static AttitudeEstimator instance;
    return instance;
}

AttitudeEstimator::~AttitudeEstimator() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t AttitudeEstimator::init(const Config& config) {
    config_ = config;

    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create attitude mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize reference vectors
    accel_ref_ = Vec3(0, 0, -9.81f);  // Gravity pointing down
    mag_ref_ = Vec3(1, 0, 0);          // North

    reset();

    initialized_ = true;
    ESP_LOGI(TAG, "AttitudeEstimator initialized (alpha=%.2f)", config_.complementary_alpha);
    return ESP_OK;
}

void AttitudeEstimator::reset() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.attitude = Quat();
    state_.gyro_bias = Vec3();
    state_.roll = 0;
    state_.pitch = 0;
    state_.yaw = 0;
    state_.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(mutex_);
}

void AttitudeEstimator::initFromAccel(const Vec3& accel) {
    Vec3 a = accel.normalized();

    // Calculate pitch and roll from accelerometer
    float pitch = std::asin(-a.x);
    float roll = std::atan2(a.y, -a.z);

    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.attitude = Quat::fromEuler(roll, pitch, 0);
    state_.roll = roll;
    state_.pitch = pitch;
    state_.yaw = 0;
    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "Attitude initialized from accel: roll=%.1f, pitch=%.1f deg",
             roll * 180.0f / M_PI, pitch * 180.0f / M_PI);
}

void AttitudeEstimator::update(const Vec3& accel, const Vec3& gyro, float dt) {
    if (!initialized_ || dt <= 0) return;

    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Remove bias from gyro
    Vec3 gyro_corrected = gyro - state_.gyro_bias;

    // Integrate gyroscope (prediction step)
    float angle = gyro_corrected.norm() * dt;
    if (angle > 1e-6f) {
        Quat dq = Quat::fromAxisAngle(gyro_corrected, angle);
        state_.attitude = state_.attitude * dq;
        state_.attitude.normalize();
    }

    // Complementary filter correction using accelerometer
    Vec3 a = accel.normalized();
    if (a.norm() > 0.5f) {  // Valid accelerometer reading
        // Calculate attitude from accelerometer
        float accel_pitch = std::asin(-a.x);
        float accel_roll = std::atan2(a.y, -a.z);

        // Get current Euler angles
        float gyro_roll, gyro_pitch, gyro_yaw;
        state_.attitude.toEuler(gyro_roll, gyro_pitch, gyro_yaw);

        // Complementary filter blend
        float alpha = config_.complementary_alpha;
        float fused_roll = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
        float fused_pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;

        // Update quaternion from fused angles
        state_.attitude = Quat::fromEuler(fused_roll, fused_pitch, gyro_yaw);
        state_.roll = fused_roll;
        state_.pitch = fused_pitch;
        state_.yaw = gyro_yaw;
    }

    state_.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(mutex_);
}

void AttitudeEstimator::updateMag(const Vec3& mag) {
    if (!initialized_ || !config_.use_mag_heading) return;

    Vec3 m = mag.normalized();
    if (m.norm() < 0.5f) return;  // Invalid reading

    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Tilt compensation: rotate mag to horizontal plane
    Vec3 m_body = state_.attitude.conjugate().rotate(m);

    // Calculate heading
    float mag_yaw = std::atan2(-m_body.y, m_body.x) + config_.mag_declination;

    // Normalize to [-pi, pi]
    while (mag_yaw > M_PI) mag_yaw -= 2.0f * M_PI;
    while (mag_yaw < -M_PI) mag_yaw += 2.0f * M_PI;

    // Blend with current yaw (low-pass)
    float alpha = 0.1f;  // Slow update for heading
    state_.yaw = alpha * mag_yaw + (1.0f - alpha) * state_.yaw;

    // Reconstruct quaternion
    state_.attitude = Quat::fromEuler(state_.roll, state_.pitch, state_.yaw);

    xSemaphoreGive(mutex_);
}

AttitudeEstimator::State AttitudeEstimator::getState() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    State s = state_;
    xSemaphoreGive(mutex_);
    return s;
}

// ============================================================================
// AltitudeEstimator Implementation
// ============================================================================

AltitudeEstimator& AltitudeEstimator::getInstance() {
    static AltitudeEstimator instance;
    return instance;
}

AltitudeEstimator::~AltitudeEstimator() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t AltitudeEstimator::init(const Config& config) {
    config_ = config;

    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create altitude mutex");
        return ESP_ERR_NO_MEM;
    }

    reset();

    initialized_ = true;
    ESP_LOGI(TAG, "AltitudeEstimator initialized");
    return ESP_OK;
}

void AltitudeEstimator::reset() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.altitude = 0;
    state_.velocity = 0;
    state_.baro_offset = 0;
    state_.timestamp_us = esp_timer_get_time();

    P_[0][0] = 1.0f; P_[0][1] = 0.0f;
    P_[1][0] = 0.0f; P_[1][1] = 1.0f;

    ground_set_ = false;
    xSemaphoreGive(mutex_);
}

void AltitudeEstimator::setGroundReference() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ground_baro_ = state_.altitude + state_.baro_offset;
    state_.baro_offset = ground_baro_;
    state_.altitude = 0;
    state_.velocity = 0;
    ground_set_ = true;
    xSemaphoreGive(mutex_);
    ESP_LOGI(TAG, "Ground reference set");
}

void AltitudeEstimator::predict(float accel_z, float dt) {
    if (!initialized_ || dt <= 0) return;

    xSemaphoreTake(mutex_, portMAX_DELAY);

    // State prediction: x = [altitude, velocity]
    // altitude += velocity * dt + 0.5 * accel * dt^2
    // velocity += accel * dt
    state_.altitude += state_.velocity * dt + 0.5f * accel_z * dt * dt;
    state_.velocity += accel_z * dt;

    // Covariance prediction
    float Q = config_.process_noise;
    float dt2 = dt * dt;

    float P00 = P_[0][0] + dt * (P_[1][0] + P_[0][1]) + dt2 * P_[1][1] + Q * dt2 * dt2 / 4.0f;
    float P01 = P_[0][1] + dt * P_[1][1] + Q * dt2 * dt / 2.0f;
    float P10 = P_[1][0] + dt * P_[1][1] + Q * dt2 * dt / 2.0f;
    float P11 = P_[1][1] + Q * dt2;

    P_[0][0] = P00; P_[0][1] = P01;
    P_[1][0] = P10; P_[1][1] = P11;

    state_.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(mutex_);
}

void AltitudeEstimator::updateBaro(float altitude) {
    if (!initialized_) return;

    xSemaphoreTake(mutex_, portMAX_DELAY);

    float z = altitude - state_.baro_offset;  // Measurement
    float R = config_.baro_sigma * config_.baro_sigma;  // Measurement noise

    // Kalman gain
    float S = P_[0][0] + R;
    float K0 = P_[0][0] / S;
    float K1 = P_[1][0] / S;

    // Update state
    float y = z - state_.altitude;  // Innovation
    state_.altitude += K0 * y;
    state_.velocity += K1 * y;

    // Update covariance
    float P00 = (1.0f - K0) * P_[0][0];
    float P01 = (1.0f - K0) * P_[0][1];
    float P10 = P_[1][0] - K1 * P_[0][0];
    float P11 = P_[1][1] - K1 * P_[0][1];

    P_[0][0] = P00; P_[0][1] = P01;
    P_[1][0] = P10; P_[1][1] = P11;

    xSemaphoreGive(mutex_);
}

void AltitudeEstimator::updateToF(float distance, float pitch, float roll) {
    if (!initialized_) return;

    // Check if ToF reading is valid
    if (distance <= 0 || distance > config_.tof_max_range) return;

    // Compensate for tilt
    float cos_tilt = std::cos(pitch) * std::cos(roll);
    if (cos_tilt < 0.5f) return;  // Too tilted, ignore

    float height = distance * cos_tilt;

    xSemaphoreTake(mutex_, portMAX_DELAY);

    float R = config_.tof_sigma * config_.tof_sigma;

    // Kalman gain
    float S = P_[0][0] + R;
    float K0 = P_[0][0] / S;
    float K1 = P_[1][0] / S;

    // Update
    float y = height - state_.altitude;
    state_.altitude += K0 * y;
    state_.velocity += K1 * y;

    // Update covariance
    P_[0][0] = (1.0f - K0) * P_[0][0];
    P_[0][1] = (1.0f - K0) * P_[0][1];
    P_[1][0] = P_[1][0] - K1 * P_[0][0];
    P_[1][1] = P_[1][1] - K1 * P_[0][1];

    xSemaphoreGive(mutex_);
}

AltitudeEstimator::State AltitudeEstimator::getState() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    State s = state_;
    xSemaphoreGive(mutex_);
    return s;
}

// ============================================================================
// VelocityEstimator Implementation
// ============================================================================

VelocityEstimator& VelocityEstimator::getInstance() {
    static VelocityEstimator instance;
    return instance;
}

VelocityEstimator::~VelocityEstimator() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t VelocityEstimator::init(const Config& config) {
    config_ = config;

    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create velocity mutex");
        return ESP_ERR_NO_MEM;
    }

    reset();

    initialized_ = true;
    ESP_LOGI(TAG, "VelocityEstimator initialized");
    return ESP_OK;
}

void VelocityEstimator::reset() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.vx = 0;
    state_.vy = 0;
    state_.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(mutex_);
}

void VelocityEstimator::updateFlow(float flow_x, float flow_y, float height,
                                    float gyro_x, float gyro_y) {
    if (!initialized_) return;

    // Use provided height or default
    float h = height;
    if (h < 0.05f) h = config_.height_for_flow;

    // Compensate for body rotation
    float flow_x_comp = flow_x - gyro_y;  // Roll rate
    float flow_y_comp = flow_y + gyro_x;  // Pitch rate

    // Convert to velocity (v = flow * height * scale)
    float vx = flow_x_comp * h * config_.flow_scale;
    float vy = flow_y_comp * h * config_.flow_scale;

    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Low-pass filter
    float alpha = 0.7f;
    state_.vx = alpha * state_.vx + (1.0f - alpha) * vx;
    state_.vy = alpha * state_.vy + (1.0f - alpha) * vy;
    state_.timestamp_us = esp_timer_get_time();

    xSemaphoreGive(mutex_);
}

VelocityEstimator::State VelocityEstimator::getState() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    State s = state_;
    xSemaphoreGive(mutex_);
    return s;
}

}  // namespace stampfly
