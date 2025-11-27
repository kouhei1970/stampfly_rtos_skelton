/**
 * @file eskf.hpp
 * @brief Attitude, Altitude, and Velocity Estimators
 *
 * AttitudeEstimator (complementary filter)
 * AltitudeEstimator (Kalman filter)
 * VelocityEstimator (optical flow)
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "filter.hpp"

namespace stampfly {

/**
 * @brief Quaternion representation
 */
struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    void normalize();
    void toEuler(float& roll, float& pitch, float& yaw) const;
};

/**
 * @brief Attitude Estimator (Complementary Filter)
 */
class AttitudeEstimator {
public:
    struct Config {
        float gyro_weight;      // Gyro weight (default: 0.98)
        float mag_declination;  // Local magnetic declination (rad)
    };

    struct State {
        Quaternion orientation;
        float roll;     // rad
        float pitch;    // rad
        float yaw;      // rad
        Vec3 gyro_bias;
    };

    AttitudeEstimator() = default;

    esp_err_t init(const Config& config);
    void update(const Vec3& accel, const Vec3& gyro, float dt);
    void updateMag(const Vec3& mag);
    State getState() const { return state_; }
    void reset();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    State state_;
};

/**
 * @brief Altitude Estimator (Kalman Filter)
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
        float altitude;     // m
        float velocity;     // m/s
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
    float P_[2][2] = {{1, 0}, {0, 1}};  // Covariance matrix
};

/**
 * @brief Velocity Estimator (Optical Flow)
 */
class VelocityEstimator {
public:
    struct Config {
        float flow_scale;   // Flow sensor scale factor
    };

    struct State {
        float vx;   // m/s (body frame)
        float vy;   // m/s (body frame)
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
