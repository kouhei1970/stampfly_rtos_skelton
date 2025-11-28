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
    CommandEntry commands_[MAX_COMMANDS] = {};
    size_t command_count_ = 0;
    char input_buffer_[MAX_CMD_LEN] = {0};
    size_t input_pos_ = 0;
};

}  // namespace stampfly
