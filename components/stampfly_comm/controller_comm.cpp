/**
 * @file controller_comm.cpp
 * @brief ESP-NOW Controller Communication Implementation (Stub)
 */

#include "controller_comm.hpp"
#include "esp_log.h"

static const char* TAG = "ControllerComm";

namespace stampfly {

esp_err_t ControllerComm::init(const Config& config)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW communication (stub)");
    ESP_LOGI(TAG, "  WiFi channel: %d", config.wifi_channel);
    config_ = config;
    // TODO: Implement WiFi and ESP-NOW initialization
    initialized_ = true;
    return ESP_OK;
}

esp_err_t ControllerComm::start()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "ESP-NOW communication started");
    // TODO: Start ESP-NOW receive
    return ESP_OK;
}

esp_err_t ControllerComm::stop()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "ESP-NOW communication stopped");
    return ESP_OK;
}

void ControllerComm::setControlCallback(ControlCallback callback)
{
    control_callback_ = callback;
}

esp_err_t ControllerComm::sendTelemetry(const TelemetryPacket& packet)
{
    if (!initialized_ || !connected_) {
        return ESP_ERR_INVALID_STATE;
    }
    // TODO: Implement ESP-NOW send
    return ESP_OK;
}

void ControllerComm::enterPairingMode()
{
    ESP_LOGI(TAG, "Entering pairing mode");
    pairing_mode_ = true;
    connected_ = false;
    // TODO: Set up pairing broadcast
}

void ControllerComm::exitPairingMode()
{
    ESP_LOGI(TAG, "Exiting pairing mode");
    pairing_mode_ = false;
}

esp_err_t ControllerComm::clearPairingFromNVS()
{
    ESP_LOGI(TAG, "Clearing pairing information from NVS");
    // TODO: Clear NVS pairing data
    for (int i = 0; i < 6; i++) {
        controller_mac_[i] = 0;
    }
    connected_ = false;
    return ESP_OK;
}

bool ControllerComm::validateChecksum(const uint8_t* data, size_t len)
{
    if (len < 2) return false;
    uint8_t sum = 0;
    for (size_t i = 0; i < len - 1; i++) {
        sum += data[i];
    }
    return sum == data[len - 1];
}

}  // namespace stampfly
