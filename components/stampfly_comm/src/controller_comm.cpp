/**
 * @file controller_comm.cpp
 * @brief ESP-NOW Communication Implementation
 */
#include "controller_comm.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>

static const char* TAG = "comm";

// NVS namespace and keys
static const char* NVS_NAMESPACE = "stampfly";
static const char* NVS_KEY_CONTROLLER_MAC = "ctrl_mac";
static const char* NVS_KEY_PAIRED = "paired";

// Broadcast MAC for pairing
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Pairing magic bytes
static const uint8_t PAIRING_MAGIC[4] = {0xAA, 0x55, 0x16, 0x88};

namespace stampfly {

// Static instance pointer for callbacks
static ControllerComm* s_instance = nullptr;

ControllerComm& ControllerComm::getInstance() {
    static ControllerComm instance;
    return instance;
}

ControllerComm::~ControllerComm() {
    stop();
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t ControllerComm::init(const Config& config) {
    if (initialized_) {
        return ESP_OK;
    }

    config_ = config;
    s_instance = this;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize WiFi in station mode
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&wifi_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set WiFi channel
    ret = esp_wifi_set_channel(config_.wifi_channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Set channel failed: %s", esp_err_to_name(ret));
    }

    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register callbacks
    ret = esp_now_register_recv_cb(onDataReceived);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register recv callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_now_register_send_cb(onDataSent);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register send callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Get own MAC address
    esp_read_mac(my_mac_, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "My MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac_[0], my_mac_[1], my_mac_[2],
             my_mac_[3], my_mac_[4], my_mac_[5]);

    initialized_ = true;
    ESP_LOGI(TAG, "ControllerComm initialized");

    return ESP_OK;
}

esp_err_t ControllerComm::start() {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (running_) {
        return ESP_OK;
    }

    // Try to load pairing from NVS
    esp_err_t ret = loadPairingFromNVS();
    if (ret == ESP_OK && controller_mac_valid_) {
        // Add controller as peer
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, controller_mac_, 6);
        peer_info.channel = config_.wifi_channel;
        peer_info.encrypt = false;

        ret = esp_now_add_peer(&peer_info);
        if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
            ESP_LOGW(TAG, "Add peer failed: %s", esp_err_to_name(ret));
        }

        pairing_state_ = PairingState::PAIRED;
        ESP_LOGI(TAG, "Loaded pairing: %02X:%02X:%02X:%02X:%02X:%02X",
                 controller_mac_[0], controller_mac_[1], controller_mac_[2],
                 controller_mac_[3], controller_mac_[4], controller_mac_[5]);
    } else {
        // Enter pairing mode
        enterPairingMode();
    }

    running_ = true;
    ESP_LOGI(TAG, "ControllerComm started");
    return ESP_OK;
}

esp_err_t ControllerComm::stop() {
    if (!running_) {
        return ESP_OK;
    }

    running_ = false;

    // Stop pairing task if running
    if (pairing_task_) {
        vTaskDelete(pairing_task_);
        pairing_task_ = nullptr;
    }

    ESP_LOGI(TAG, "ControllerComm stopped");
    return ESP_OK;
}

void ControllerComm::setControlCallback(ControlCallback callback) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    control_callback_ = callback;
    xSemaphoreGive(mutex_);
}

esp_err_t ControllerComm::sendTelemetry(const TelemetryPacket& packet) {
    if (!isConnected()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare packet with checksum
    TelemetryPacket tx_packet = packet;
    tx_packet.sequence = sequence_++;
    tx_packet.checksum = calculateChecksum(
        reinterpret_cast<const uint8_t*>(&tx_packet),
        sizeof(TelemetryPacket) - 1
    );

    // Send via ESP-NOW
    esp_err_t ret = esp_now_send(controller_mac_,
                                 reinterpret_cast<const uint8_t*>(&tx_packet),
                                 sizeof(TelemetryPacket));
    return ret;
}

bool ControllerComm::isConnected() const {
    if (!controller_mac_valid_) {
        return false;
    }
    int64_t now = esp_timer_get_time();
    return (now - last_received_us_) < config_.connection_timeout_us;
}

int64_t ControllerComm::getTimeSinceLastPacket() const {
    return esp_timer_get_time() - last_received_us_;
}

void ControllerComm::enterPairingMode() {
    pairing_state_ = PairingState::WAITING;
    controller_mac_valid_ = false;

    // Add broadcast peer for pairing
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, BROADCAST_MAC, 6);
    peer_info.channel = config_.wifi_channel;
    peer_info.encrypt = false;

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Add broadcast peer failed: %s", esp_err_to_name(ret));
    }

    // Start pairing task
    if (!pairing_task_) {
        xTaskCreate(pairingTask, "pairing", 2048, this, 5, &pairing_task_);
    }

    ESP_LOGI(TAG, "Entered pairing mode");
}

esp_err_t ControllerComm::clearPairingFromNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    nvs_erase_key(handle, NVS_KEY_CONTROLLER_MAC);
    nvs_erase_key(handle, NVS_KEY_PAIRED);
    ret = nvs_commit(handle);
    nvs_close(handle);

    controller_mac_valid_ = false;
    pairing_state_ = PairingState::IDLE;

    ESP_LOGI(TAG, "Pairing cleared from NVS");
    return ret;
}

void ControllerComm::getMacAddress(uint8_t* mac) const {
    memcpy(mac, my_mac_, 6);
}

bool ControllerComm::getControllerMac(uint8_t* mac) const {
    if (!controller_mac_valid_) {
        return false;
    }
    memcpy(mac, controller_mac_, 6);
    return true;
}

// Static ESP-NOW receive callback
void ControllerComm::onDataReceived(const esp_now_recv_info_t* info,
                                    const uint8_t* data, int len) {
    if (!s_instance || !s_instance->running_) {
        return;
    }

    const uint8_t* mac = info->src_addr;

    // Check for control packet
    if (len == sizeof(ControlPacket)) {
        const ControlPacket* packet = reinterpret_cast<const ControlPacket*>(data);
        s_instance->handleControlPacket(mac, packet);
    }
    // Check for pairing response (any packet from controller during pairing)
    else if (s_instance->pairing_state_ == PairingState::WAITING && len > 0) {
        s_instance->handlePairingResponse(mac);
    }
}

// Static ESP-NOW send callback
void ControllerComm::onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        if (s_instance) {
            s_instance->error_count_++;
        }
    }
}

void ControllerComm::handleControlPacket(const uint8_t* mac, const ControlPacket* packet) {
    // Verify target MAC (lower 3 bytes)
    if (packet->drone_mac[0] != my_mac_[3] ||
        packet->drone_mac[1] != my_mac_[4] ||
        packet->drone_mac[2] != my_mac_[5]) {
        return;  // Not for us
    }

    // Verify checksum
    uint8_t calc_checksum = calculateChecksum(
        reinterpret_cast<const uint8_t*>(packet),
        sizeof(ControlPacket) - 1
    );
    if (calc_checksum != packet->checksum) {
        error_count_++;
        ESP_LOGW(TAG, "Checksum error: calc=%02X, recv=%02X",
                 calc_checksum, packet->checksum);
        return;
    }

    // Update state
    xSemaphoreTake(mutex_, portMAX_DELAY);

    last_received_us_ = esp_timer_get_time();
    packet_count_++;

    // Save controller MAC if not yet paired
    if (!controller_mac_valid_) {
        memcpy(controller_mac_, mac, 6);
        controller_mac_valid_ = true;
        pairing_state_ = PairingState::PAIRED;

        // Add as peer
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, mac, 6);
        peer_info.channel = config_.wifi_channel;
        peer_info.encrypt = false;
        esp_now_add_peer(&peer_info);

        // Save to NVS
        savePairingToNVS();

        ESP_LOGI(TAG, "Paired with controller: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // Call callback
    if (control_callback_) {
        control_callback_(*packet);
    }

    xSemaphoreGive(mutex_);
}

void ControllerComm::handlePairingResponse(const uint8_t* mac) {
    xSemaphoreTake(mutex_, portMAX_DELAY);

    if (pairing_state_ == PairingState::WAITING) {
        memcpy(controller_mac_, mac, 6);
        controller_mac_valid_ = true;
        pairing_state_ = PairingState::PAIRED;

        // Add as peer
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, mac, 6);
        peer_info.channel = config_.wifi_channel;
        peer_info.encrypt = false;
        esp_now_add_peer(&peer_info);

        // Save to NVS
        savePairingToNVS();

        // Stop pairing task
        if (pairing_task_) {
            vTaskDelete(pairing_task_);
            pairing_task_ = nullptr;
        }

        ESP_LOGI(TAG, "Pairing complete: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    xSemaphoreGive(mutex_);
}

esp_err_t ControllerComm::sendPairingBroadcast() {
    PairingPacket packet = {};
    packet.channel = config_.wifi_channel;
    memcpy(packet.drone_mac, my_mac_, 6);
    memcpy(packet.magic, PAIRING_MAGIC, 4);

    return esp_now_send(BROADCAST_MAC,
                       reinterpret_cast<const uint8_t*>(&packet),
                       sizeof(PairingPacket));
}

esp_err_t ControllerComm::savePairingToNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(handle, NVS_KEY_CONTROLLER_MAC, controller_mac_, 6);
    if (ret == ESP_OK) {
        ret = nvs_set_u8(handle, NVS_KEY_PAIRED, 1);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    return ret;
}

esp_err_t ControllerComm::loadPairingFromNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t paired = 0;
    ret = nvs_get_u8(handle, NVS_KEY_PAIRED, &paired);
    if (ret != ESP_OK || paired == 0) {
        nvs_close(handle);
        return ESP_ERR_NOT_FOUND;
    }

    size_t len = 6;
    ret = nvs_get_blob(handle, NVS_KEY_CONTROLLER_MAC, controller_mac_, &len);
    nvs_close(handle);

    if (ret == ESP_OK && len == 6) {
        controller_mac_valid_ = true;
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

uint8_t ControllerComm::calculateChecksum(const uint8_t* data, size_t len) const {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

void ControllerComm::pairingTask(void* param) {
    ControllerComm* self = static_cast<ControllerComm*>(param);

    while (self->pairing_state_ == PairingState::WAITING) {
        self->sendPairingBroadcast();
        vTaskDelay(pdMS_TO_TICKS(PAIRING_INTERVAL_MS));
    }

    self->pairing_task_ = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace stampfly
