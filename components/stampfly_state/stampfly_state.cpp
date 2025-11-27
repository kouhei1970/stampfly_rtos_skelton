/**
 * @file stampfly_state.cpp
 * @brief StampFly State Manager Implementation
 */

#include "stampfly_state.hpp"
#include "esp_log.h"

static const char* TAG = "StampFlyState";

namespace stampfly {

StampFlyState& StampFlyState::getInstance()
{
    static StampFlyState instance;
    return instance;
}

esp_err_t StampFlyState::init()
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "StampFlyState initialized");
    return ESP_OK;
}

FlightState StampFlyState::getFlightState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    FlightState state = flight_state_;
    xSemaphoreGive(mutex_);
    return state;
}

PairingState StampFlyState::getPairingState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    PairingState state = pairing_state_;
    xSemaphoreGive(mutex_);
    return state;
}

ErrorCode StampFlyState::getErrorCode() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ErrorCode code = error_code_;
    xSemaphoreGive(mutex_);
    return code;
}

bool StampFlyState::requestArm()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool success = false;
    if (flight_state_ == FlightState::IDLE && error_code_ == ErrorCode::NONE) {
        flight_state_ = FlightState::ARMED;
        success = true;
        ESP_LOGI(TAG, "Armed");
    }
    xSemaphoreGive(mutex_);
    return success;
}

bool StampFlyState::requestDisarm()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool success = false;
    if (flight_state_ == FlightState::ARMED || flight_state_ == FlightState::FLYING) {
        flight_state_ = FlightState::IDLE;
        success = true;
        ESP_LOGI(TAG, "Disarmed");
    }
    xSemaphoreGive(mutex_);
    return success;
}

void StampFlyState::setFlightState(FlightState state)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    flight_state_ = state;
    xSemaphoreGive(mutex_);
}

void StampFlyState::setPairingState(PairingState state)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    pairing_state_ = state;
    xSemaphoreGive(mutex_);
}

void StampFlyState::setError(ErrorCode code)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_code_ = code;
    if (code != ErrorCode::NONE) {
        flight_state_ = FlightState::ERROR;
    }
    xSemaphoreGive(mutex_);
}

void StampFlyState::clearError()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_code_ = ErrorCode::NONE;
    xSemaphoreGive(mutex_);
}

// Sensor data getters
void StampFlyState::getIMUData(Vec3& accel, Vec3& gyro) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    accel = accel_;
    gyro = gyro_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getMagData(Vec3& mag) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    mag = mag_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getBaroData(float& altitude, float& pressure) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    altitude = baro_altitude_;
    pressure = baro_pressure_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getToFData(float& bottom, float& front) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bottom = tof_bottom_;
    front = tof_front_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getFlowData(float& vx, float& vy) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    vx = flow_vx_;
    vy = flow_vy_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getPowerData(float& voltage, float& current) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    voltage = power_voltage_;
    current = power_current_;
    xSemaphoreGive(mutex_);
}

// Sensor data setters
void StampFlyState::updateIMU(const Vec3& accel, const Vec3& gyro)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    accel_ = accel;
    gyro_ = gyro;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateMag(const Vec3& mag)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    mag_ = mag;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateBaro(float altitude, float pressure)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    baro_altitude_ = altitude;
    baro_pressure_ = pressure;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateToF(float bottom, float front)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    tof_bottom_ = bottom;
    tof_front_ = front;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateFlow(float vx, float vy)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    flow_vx_ = vx;
    flow_vy_ = vy;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updatePower(float voltage, float current)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    power_voltage_ = voltage;
    power_current_ = current;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getAttitude(float& roll, float& pitch, float& yaw) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateAttitude(float roll, float pitch, float yaw)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getControlInput(float& throttle, float& roll, float& pitch, float& yaw) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    throttle = ctrl_throttle_;
    roll = ctrl_roll_;
    pitch = ctrl_pitch_;
    yaw = ctrl_yaw_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateControlInput(float throttle, float roll, float pitch, float yaw)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ctrl_throttle_ = throttle;
    ctrl_roll_ = roll;
    ctrl_pitch_ = pitch;
    ctrl_yaw_ = yaw;
    xSemaphoreGive(mutex_);
}

}  // namespace stampfly
