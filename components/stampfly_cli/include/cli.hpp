/**
 * @file cli.hpp
 * @brief CLI Console (USB CDC)
 *
 * Command processing, sensor display, calibration
 */

#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include "esp_err.h"

namespace stampfly {

using CommandHandler = std::function<void(int argc, char** argv)>;

class CLI {
public:
    static constexpr size_t MAX_CMD_LEN = 128;
    static constexpr size_t MAX_ARGS = 8;

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
     * @param handler Command handler function
     * @param help Help text
     */
    void registerCommand(const char* name, CommandHandler handler, const char* help = "");

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

private:
    struct Command {
        CommandHandler handler;
        std::string help;
    };

    void parseAndExecute(const char* line);
    void printHelp();

    bool initialized_ = false;
    std::map<std::string, Command> commands_;
    char input_buffer_[MAX_CMD_LEN] = {0};
    size_t input_pos_ = 0;
};

}  // namespace stampfly
