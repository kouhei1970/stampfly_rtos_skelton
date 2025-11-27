/**
 * @file stampfly_state.cpp
 * @brief StampFly State Management Implementation
 */

#include "stampfly_state.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cmath>

static const char* TAG = "state";

namespace stampfly {

StampFlyState& StampFlyState::getInstance() {
    static StampFlyState instance;
    return instance;
}

StampFlyState::StampFlyState() {
    mutex_ = xSemaphoreCreateMutex();
    configASSERT(mutex_ != nullptr);
}

StampFlyState::~StampFlyState() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

// State getters
StampFlyState::FlightState StampFlyState::getFlightState() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    FlightState state = flight_state_;
    xSemaphoreGive(mutex_);
    return state;
}

StampFlyState::PairingState StampFlyState::getPairingState() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    PairingState state = pairing_state_;
    xSemaphoreGive(mutex_);
    return state;
}

StampFlyState::ErrorCode StampFlyState::getErrorCode() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ErrorCode code = error_code_;
    xSemaphoreGive(mutex_);
    return code;
}

bool StampFlyState::hasWarning(WarningFlags flag) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool result = (warning_flags_ & flag) != 0;
    xSemaphoreGive(mutex_);
    return result;
}

bool StampFlyState::hasError() const {
    return getErrorCode() != ErrorCode::NONE;
}

// Sensor data getters
void StampFlyState::getIMUData(Vector3& accel, Vector3& gyro) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    accel = sensor_data_.accel;
    gyro = sensor_data_.gyro;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getMagData(Vector3& mag) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    mag = sensor_data_.mag;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getBaroData(float& altitude, float& pressure) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    altitude = sensor_data_.baro_altitude;
    pressure = sensor_data_.baro_pressure;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getToFData(float& front, float& bottom) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    front = sensor_data_.tof_front;
    bottom = sensor_data_.tof_bottom;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getFlowData(Vector2& velocity) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    velocity = sensor_data_.flow_velocity;
    xSemaphoreGive(mutex_);
}

// Estimated state getters
void StampFlyState::getPosition(Vector3& pos) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    pos = estimated_state_.position;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getVelocity(Vector3& vel) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    vel = estimated_state_.velocity;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getAttitude(Quaternion& quat) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    quat = estimated_state_.attitude;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getEulerAngles(float& roll, float& pitch, float& yaw) const {
    Quaternion q;
    getAttitude(q);

    // Convert quaternion to Euler angles
    roll = std::atan2(2.0f * (q.w * q.x + q.y * q.z),
                      1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    pitch = std::asin(2.0f * (q.w * q.y - q.z * q.x));
    yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                     1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

// Controller input
void StampFlyState::getControlInput(ControlInput& input) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    input = control_input_;
    xSemaphoreGive(mutex_);
}

uint16_t StampFlyState::getThrottleInput() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    uint16_t throttle = control_input_.throttle;
    xSemaphoreGive(mutex_);
    return throttle;
}

// Sensor data setters
void StampFlyState::updateIMU(const Vector3& accel, const Vector3& gyro) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sensor_data_.accel = accel;
    sensor_data_.gyro = gyro;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateMag(const Vector3& mag) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sensor_data_.mag = mag;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateBaro(float altitude, float pressure) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sensor_data_.baro_altitude = altitude;
    sensor_data_.baro_pressure = pressure;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateToF(float front, float bottom) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sensor_data_.tof_front = front;
    sensor_data_.tof_bottom = bottom;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateFlow(const Vector2& velocity) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sensor_data_.flow_velocity = velocity;
    xSemaphoreGive(mutex_);
}

// Battery state
void StampFlyState::updateBattery(float voltage, float current) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    battery_data_.voltage = voltage;
    battery_data_.current = current;
    xSemaphoreGive(mutex_);
}

float StampFlyState::getBatteryVoltage() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    float voltage = battery_data_.voltage;
    xSemaphoreGive(mutex_);
    return voltage;
}

float StampFlyState::getBatteryCurrent() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    float current = battery_data_.current;
    xSemaphoreGive(mutex_);
    return current;
}

float StampFlyState::getBatteryPercent() const {
    float voltage = getBatteryVoltage();
    constexpr float MIN_V = 3.3f;
    constexpr float MAX_V = 4.2f;
    float percent = (voltage - MIN_V) / (MAX_V - MIN_V) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

// Warning flag operations
void StampFlyState::setWarning(WarningFlags flag) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    warning_flags_ |= flag;
    xSemaphoreGive(mutex_);
}

void StampFlyState::clearWarning(WarningFlags flag) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    warning_flags_ &= ~flag;
    xSemaphoreGive(mutex_);
}

// State transitions
void StampFlyState::setFlightState(FlightState state) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ESP_LOGI(TAG, "Flight state: %d -> %d", (int)flight_state_, (int)state);
    flight_state_ = state;
    xSemaphoreGive(mutex_);
}

void StampFlyState::setPairingState(PairingState state) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ESP_LOGI(TAG, "Pairing state: %d -> %d", (int)pairing_state_, (int)state);
    pairing_state_ = state;
    xSemaphoreGive(mutex_);
}

bool StampFlyState::requestArm() {
    xSemaphoreTake(mutex_, portMAX_DELAY);

    bool success = false;

    // Can only arm from IDLE state
    if (flight_state_ == FlightState::IDLE && error_code_ == ErrorCode::NONE) {
        flight_state_ = FlightState::ARMED;
        success = true;
        ESP_LOGI(TAG, "Armed");
    } else {
        ESP_LOGW(TAG, "Arm rejected: state=%d, error=%d",
                 (int)flight_state_, (int)error_code_);
    }

    xSemaphoreGive(mutex_);
    return success;
}

bool StampFlyState::requestDisarm() {
    xSemaphoreTake(mutex_, portMAX_DELAY);

    bool success = false;

    // Can disarm from ARMED or FLYING state
    if (flight_state_ == FlightState::ARMED || flight_state_ == FlightState::FLYING) {
        flight_state_ = FlightState::IDLE;
        success = true;
        ESP_LOGI(TAG, "Disarmed");
    }

    xSemaphoreGive(mutex_);
    return success;
}

void StampFlyState::setError(ErrorCode code) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_code_ = code;
    if (code != ErrorCode::NONE) {
        flight_state_ = FlightState::ERROR;
        ESP_LOGE(TAG, "Error set: %d", (int)code);
    }
    xSemaphoreGive(mutex_);
}

void StampFlyState::clearError() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_code_ = ErrorCode::NONE;
    if (flight_state_ == FlightState::ERROR) {
        flight_state_ = FlightState::IDLE;
    }
    xSemaphoreGive(mutex_);
}

// NVS persistence
esp_err_t StampFlyState::saveToNVS() {
    // TODO: Implement calibration data save
    return ESP_OK;
}

esp_err_t StampFlyState::loadFromNVS() {
    // TODO: Implement calibration data load
    return ESP_OK;
}

}  // namespace stampfly
