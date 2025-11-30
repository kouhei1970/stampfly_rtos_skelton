/**
 * @file cli.hpp
 * @brief CLI Console (USB CDC)
 *
 * Command processing, sensor display, calibration
 *
 * Note: Uses static arrays instead of std::map/std::function
 * to avoid dynamic memory allocation issues on embedded systems.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include "esp_err.h"

namespace stampfly {

/**
 * @brief Binary log packet structure (64 bytes)
 *
 * Used for ESKF debugging - all sensor data in one packet
 */
#pragma pack(push, 1)
struct BinaryLogPacket {
    uint8_t header[2];      // 0xAA, 0x55
    uint32_t timestamp_ms;  // ms since boot
    float accel_x;          // [m/s²]
    float accel_y;
    float accel_z;
    float gyro_x;           // [rad/s]
    float gyro_y;
    float gyro_z;
    float mag_x;            // [uT]
    float mag_y;
    float mag_z;
    float pressure;         // [Pa]
    float baro_alt;         // [m]
    float tof_bottom;       // [m]
    float tof_front;        // [m]
    int16_t flow_dx;        // raw delta
    int16_t flow_dy;
    uint8_t flow_squal;     // surface quality
    uint8_t checksum;       // XOR of bytes 2-62
};
#pragma pack(pop)

static_assert(sizeof(BinaryLogPacket) == 64, "BinaryLogPacket must be 64 bytes");

/**
 * @brief Command handler function pointer type
 */
using CommandHandlerFn = void (*)(int argc, char** argv, void* context);

class CLI {
public:
    static constexpr size_t MAX_CMD_LEN = 128;
    static constexpr size_t MAX_ARGS = 8;
    static constexpr size_t MAX_COMMANDS = 16;
    static constexpr size_t MAX_CMD_NAME_LEN = 16;
    static constexpr size_t MAX_HELP_LEN = 48;

    CLI() = default;
    ~CLI() = default;

    /**
     * @brief Initialize CLI
     * @return ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief Register command handler
     * @param name Command name
     * @param handler Command handler function pointer
     * @param help Help text
     * @param context Optional context pointer passed to handler
     */
    void registerCommand(const char* name, CommandHandlerFn handler,
                         const char* help = "", void* context = nullptr);

    /**
     * @brief Process input (call periodically)
     */
    void processInput();

    /**
     * @brief Print to CLI output
     * @param format Printf format string
     */
    void print(const char* format, ...);

    /**
     * @brief Register default commands
     */
    void registerDefaultCommands();

    bool isInitialized() const { return initialized_; }

    /**
     * @brief Print help (available commands)
     */
    void printHelp();

    /**
     * @brief Check if teleplot streaming is enabled
     */
    bool isTeleplotEnabled() const { return teleplot_enabled_; }

    /**
     * @brief Set teleplot streaming state
     */
    void setTeleplotEnabled(bool enabled) { teleplot_enabled_ = enabled; }

    /**
     * @brief Output teleplot data (call periodically from task)
     */
    void outputTeleplot();

    /**
     * @brief Check if CSV logging is enabled
     */
    bool isLogEnabled() const { return log_enabled_; }

    /**
     * @brief Set CSV logging state
     */
    void setLogEnabled(bool enabled) { log_enabled_ = enabled; }

    /**
     * @brief Output CSV log data (call periodically from task)
     */
    void outputCSVLog();

    /**
     * @brief Get log sample counter
     */
    uint32_t getLogCounter() const { return log_counter_; }

    /**
     * @brief Reset log sample counter
     */
    void resetLogCounter() { log_counter_ = 0; }

    /**
     * @brief Check if binary logging is enabled
     */
    bool isBinlogEnabled() const { return binlog_enabled_; }

    /**
     * @brief Set binary logging state
     */
    void setBinlogEnabled(bool enabled) { binlog_enabled_ = enabled; }

    /**
     * @brief Output binary log packet (call periodically from task at 100Hz)
     */
    void outputBinaryLog();

    /**
     * @brief Get binary log sample counter
     */
    uint32_t getBinlogCounter() const { return binlog_counter_; }

    /**
     * @brief Reset binary log sample counter
     */
    void resetBinlogCounter() { binlog_counter_ = 0; }

private:
    /**
     * @brief Static command entry (no dynamic allocation)
     */
    struct CommandEntry {
        char name[MAX_CMD_NAME_LEN];
        char help[MAX_HELP_LEN];
        CommandHandlerFn handler;
        void* context;
    };

    void parseAndExecute(const char* line);

    bool initialized_ = false;
    bool teleplot_enabled_ = false;
    bool log_enabled_ = false;
    bool binlog_enabled_ = false;
    uint32_t log_counter_ = 0;
    uint32_t binlog_counter_ = 0;
    CommandEntry commands_[MAX_COMMANDS] = {};
    size_t command_count_ = 0;
    char input_buffer_[MAX_CMD_LEN] = {0};
    size_t input_pos_ = 0;
};

}  // namespace stampfly
