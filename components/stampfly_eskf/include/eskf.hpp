/**
 * @file eskf.hpp
 * @brief Attitude/Position Estimator
 *
 * This module provides sensor fusion for StampFly:
 * - Complementary filter for attitude estimation (IMU + Mag)
 * - Kalman filter for altitude estimation (Baro + ToF + IMU)
 * - Optical flow integration for horizontal velocity
 *
 * Future: Integrate full ESKF from kouhei1970/stampfly-eskf-estimator
 */
#pragma once
#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

// 3D Vector
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    float norm() const;
    Vec3 normalized() const;
};

// Quaternion for rotation
struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator*(const Quat& q) const;
    Quat conjugate() const { return Quat(w, -x, -y, -z); }
    float norm() const;
    void normalize();
    Vec3 rotate(const Vec3& v) const;
    void toEuler(float& roll, float& pitch, float& yaw) const;
    static Quat fromEuler(float roll, float pitch, float yaw);
    static Quat fromAxisAngle(const Vec3& axis, float angle);
};

class AttitudeEstimator {
public:
    struct Config {
        float complementary_alpha;  // Gyro weight (0-1)
        float gyro_bias_alpha;      // Bias estimation rate
        float mag_declination;      // Local magnetic declination [rad]
        bool use_mag_heading;       // Use magnetometer for yaw

        Config() :
            complementary_alpha(0.98f),
            gyro_bias_alpha(0.999f),
            mag_declination(0.0f),
            use_mag_heading(true)
        {}
    };

    struct State {
        Quat attitude;          // Quaternion attitude
        Vec3 gyro_bias;         // Gyro bias estimate [rad/s]
        float roll = 0.0f;      // Roll angle [rad]
        float pitch = 0.0f;     // Pitch angle [rad]
        float yaw = 0.0f;       // Yaw angle [rad]
        int64_t timestamp_us = 0;
    };

    static AttitudeEstimator& getInstance();

    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Update with IMU data
     * @param accel Accelerometer [m/s^2]
     * @param gyro Gyroscope [rad/s]
     * @param dt Time step [s]
     */
    void update(const Vec3& accel, const Vec3& gyro, float dt);

    /**
     * @brief Update magnetometer (for yaw correction)
     * @param mag Magnetometer [uT]
     */
    void updateMag(const Vec3& mag);

    /**
     * @brief Get current state
     */
    State getState() const;

    /**
     * @brief Reset estimator to initial state
     */
    void reset();

    /**
     * @brief Set initial attitude from accelerometer
     */
    void initFromAccel(const Vec3& accel);

private:
    AttitudeEstimator() = default;
    ~AttitudeEstimator();

    Config config_;
    State state_;
    bool initialized_ = false;
    mutable SemaphoreHandle_t mutex_ = nullptr;

    Vec3 accel_ref_;  // Reference gravity vector
    Vec3 mag_ref_;    // Reference magnetic field
};

class AltitudeEstimator {
public:
    struct Config {
        float baro_sigma;           // Barometer noise [m]
        float tof_sigma;            // ToF noise [m]
        float accel_sigma;          // Accelerometer noise [m/s^2]
        float tof_max_range;        // ToF valid range [m]
        float process_noise;        // Process noise

        Config() :
            baro_sigma(0.5f),
            tof_sigma(0.02f),
            accel_sigma(0.1f),
            tof_max_range(2.0f),
            process_noise(0.01f)
        {}
    };

    struct State {
        float altitude = 0.0f;      // Altitude [m]
        float velocity = 0.0f;      // Vertical velocity [m/s]
        float baro_offset = 0.0f;   // Barometer ground offset
        int64_t timestamp_us = 0;
    };

    static AltitudeEstimator& getInstance();

    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Predict step with acceleration
     * @param accel_z Vertical acceleration (body frame) [m/s^2]
     * @param dt Time step [s]
     */
    void predict(float accel_z, float dt);

    /**
     * @brief Update with barometer
     * @param altitude Barometric altitude [m]
     */
    void updateBaro(float altitude);

    /**
     * @brief Update with ToF sensor
     * @param distance Distance to ground [m]
     * @param pitch Current pitch angle [rad]
     * @param roll Current roll angle [rad]
     */
    void updateToF(float distance, float pitch, float roll);

    /**
     * @brief Set ground reference (current position as zero)
     */
    void setGroundReference();

    State getState() const;
    void reset();

private:
    AltitudeEstimator() = default;
    ~AltitudeEstimator();

    Config config_;
    State state_;
    bool initialized_ = false;
    mutable SemaphoreHandle_t mutex_ = nullptr;

    // Kalman filter state
    float P_[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};  // Covariance
    float ground_baro_ = 0.0f;
    bool ground_set_ = false;
};

class VelocityEstimator {
public:
    struct Config {
        float flow_scale;           // Optical flow scale factor
        float height_for_flow;      // Default height for flow conversion

        Config() :
            flow_scale(1.0f),
            height_for_flow(0.5f)
        {}
    };

    struct State {
        float vx = 0.0f;           // X velocity [m/s]
        float vy = 0.0f;           // Y velocity [m/s]
        int64_t timestamp_us = 0;
    };

    static VelocityEstimator& getInstance();

    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Update with optical flow
     * @param flow_x Flow X [rad/s]
     * @param flow_y Flow Y [rad/s]
     * @param height Current height [m]
     * @param gyro_x Roll rate [rad/s]
     * @param gyro_y Pitch rate [rad/s]
     */
    void updateFlow(float flow_x, float flow_y, float height,
                    float gyro_x, float gyro_y);

    State getState() const;
    void reset();

private:
    VelocityEstimator() = default;
    ~VelocityEstimator();

    Config config_;
    State state_;
    bool initialized_ = false;
    mutable SemaphoreHandle_t mutex_ = nullptr;
};

}  // namespace stampfly
