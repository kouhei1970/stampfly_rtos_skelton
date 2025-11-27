/**
 * @file controller_comm.hpp
 * @brief ESP-NOW Communication with Controller
 *
 * Handles ESP-NOW communication between StampFly and controller.
 * - Receives control packets from controller
 * - Sends telemetry packets to controller
 * - Manages pairing state
 */
#pragma once

#include <cstdint>
#include <functional>
#include "esp_err.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

namespace stampfly {

class ControllerComm {
public:
    // Control packet from controller (12 bytes)
    struct ControlPacket {
        uint8_t drone_mac[3];   // Target drone MAC (lower 3 bytes)
        uint16_t throttle;      // 0-1000
        uint16_t roll;          // 0-1000 (500 = center)
        uint16_t pitch;         // 0-1000 (500 = center)
        uint16_t yaw;           // 0-1000 (500 = center)
        uint8_t flags;          // bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode
        uint8_t checksum;
    } __attribute__((packed));

    // Telemetry packet to controller (20 bytes)
    struct TelemetryPacket {
        uint8_t packet_type;    // 0x01 = telemetry
        uint8_t sequence;
        uint16_t battery_mv;    // Battery voltage [mV]
        int16_t altitude_cm;    // Altitude [cm]
        int16_t velocity_x;     // X velocity [mm/s]
        int16_t velocity_y;     // Y velocity [mm/s]
        int16_t velocity_z;     // Z velocity [mm/s]
        int16_t roll_deg10;     // Roll [0.1 deg]
        int16_t pitch_deg10;    // Pitch [0.1 deg]
        int16_t yaw_deg10;      // Yaw [0.1 deg]
        uint8_t state;          // FlightState
        uint8_t error;          // ErrorCode
        uint8_t checksum;
    } __attribute__((packed));

    // Pairing packet (14 bytes)
    struct PairingPacket {
        uint8_t channel;        // WiFi channel
        uint8_t drone_mac[6];   // Full MAC address
        uint8_t magic[4];       // 0xAA, 0x55, 0x16, 0x88
        uint8_t reserved[3];
    } __attribute__((packed));

    // Pairing states
    enum class PairingState {
        IDLE,       // Normal operation (not connected)
        WAITING,    // Waiting for pairing
        PAIRED      // Paired with controller
    };

    // Control flags
    static constexpr uint8_t FLAG_ARM = (1 << 0);
    static constexpr uint8_t FLAG_FLIP = (1 << 1);
    static constexpr uint8_t FLAG_MODE = (1 << 2);
    static constexpr uint8_t FLAG_ALT_MODE = (1 << 3);

    // Timing constants
    static constexpr int64_t CONNECTION_TIMEOUT_US = 500000;  // 500ms
    static constexpr int64_t PAIRING_INTERVAL_MS = 200;       // 200ms
    static constexpr uint8_t WIFI_CHANNEL = 1;

    struct Config {
        uint8_t wifi_channel;
        int64_t connection_timeout_us;

        Config() : wifi_channel(WIFI_CHANNEL), connection_timeout_us(CONNECTION_TIMEOUT_US) {}
    };

    using ControlCallback = std::function<void(const ControlPacket&)>;

    static ControllerComm& getInstance();

    /**
     * @brief Initialize ESP-NOW communication
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Start communication (load pairing from NVS or enter pairing mode)
     * @return ESP_OK on success
     */
    esp_err_t start();

    /**
     * @brief Stop communication
     * @return ESP_OK on success
     */
    esp_err_t stop();

    /**
     * @brief Set control packet callback
     * @param callback Function called when control packet received
     */
    void setControlCallback(ControlCallback callback);

    /**
     * @brief Send telemetry packet to controller
     * @param packet Telemetry data
     * @return ESP_OK on success
     */
    esp_err_t sendTelemetry(const TelemetryPacket& packet);

    /**
     * @brief Check if connected to controller
     * @return true if connected (received packet within timeout)
     */
    bool isConnected() const;

    /**
     * @brief Get time since last received packet
     * @return Time in microseconds
     */
    int64_t getTimeSinceLastPacket() const;

    /**
     * @brief Get current pairing state
     */
    PairingState getPairingState() const { return pairing_state_; }

    /**
     * @brief Enter pairing mode
     */
    void enterPairingMode();

    /**
     * @brief Clear pairing from NVS
     * @return ESP_OK on success
     */
    esp_err_t clearPairingFromNVS();

    /**
     * @brief Get this device's MAC address
     * @param mac Output buffer (6 bytes)
     */
    void getMacAddress(uint8_t* mac) const;

    /**
     * @brief Get controller MAC address
     * @param mac Output buffer (6 bytes)
     * @return true if paired
     */
    bool getControllerMac(uint8_t* mac) const;

    /**
     * @brief Get receive statistics
     */
    uint32_t getPacketCount() const { return packet_count_; }
    uint32_t getErrorCount() const { return error_count_; }

private:
    ControllerComm() = default;
    ~ControllerComm();
    ControllerComm(const ControllerComm&) = delete;
    ControllerComm& operator=(const ControllerComm&) = delete;

    // ESP-NOW callbacks (static)
    static void onDataReceived(const esp_now_recv_info_t* info,
                               const uint8_t* data, int len);
    static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);

    // Internal methods
    void handleControlPacket(const uint8_t* mac, const ControlPacket* packet);
    void handlePairingResponse(const uint8_t* mac);
    esp_err_t sendPairingBroadcast();
    esp_err_t savePairingToNVS();
    esp_err_t loadPairingFromNVS();
    uint8_t calculateChecksum(const uint8_t* data, size_t len) const;
    static void pairingTask(void* param);

    Config config_;
    bool initialized_ = false;
    bool running_ = false;
    PairingState pairing_state_ = PairingState::IDLE;

    uint8_t my_mac_[6] = {0};
    uint8_t controller_mac_[6] = {0};
    bool controller_mac_valid_ = false;

    int64_t last_received_us_ = 0;
    uint32_t packet_count_ = 0;
    uint32_t error_count_ = 0;
    uint8_t sequence_ = 0;

    ControlCallback control_callback_;
    mutable SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t pairing_task_ = nullptr;
};

}  // namespace stampfly
