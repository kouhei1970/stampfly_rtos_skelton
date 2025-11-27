/**
 * @file cli.cpp
 * @brief CLI Console Implementation (Stub)
 */

#include "cli.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>

static const char* TAG = "CLI";

namespace stampfly {

esp_err_t CLI::init()
{
    ESP_LOGI(TAG, "Initializing CLI (stub)");
    // TODO: Initialize USB CDC input
    initialized_ = true;
    return ESP_OK;
}

void CLI::registerCommand(const char* name, CommandHandler handler, const char* help)
{
    commands_[name] = {handler, help ? help : ""};
}

void CLI::processInput()
{
    if (!initialized_) return;
    // TODO: Read from USB CDC and process commands
}

void CLI::print(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

void CLI::parseAndExecute(const char* line)
{
    char buffer[MAX_CMD_LEN];
    strncpy(buffer, line, MAX_CMD_LEN - 1);
    buffer[MAX_CMD_LEN - 1] = '\0';

    char* argv[MAX_ARGS];
    int argc = 0;

    char* token = strtok(buffer, " \t\n");
    while (token != nullptr && argc < MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(nullptr, " \t\n");
    }

    if (argc == 0) return;

    auto it = commands_.find(argv[0]);
    if (it != commands_.end()) {
        it->second.handler(argc, argv);
    } else {
        print("Unknown command: %s\n", argv[0]);
        print("Type 'help' for available commands\n");
    }
}

void CLI::printHelp()
{
    print("\nAvailable commands:\n");
    for (const auto& cmd : commands_) {
        print("  %-12s %s\n", cmd.first.c_str(), cmd.second.help.c_str());
    }
    print("\n");
}

void CLI::registerDefaultCommands()
{
    registerCommand("help", [this](int, char**) {
        printHelp();
    }, "Show available commands");

    registerCommand("status", [this](int, char**) {
        print("System status (stub)\n");
        // TODO: Show flight state, battery, connection status
    }, "Show system status");

    registerCommand("sensor", [this](int argc, char** argv) {
        if (argc < 2) {
            print("Usage: sensor [imu|mag|baro|tof|flow|power]\n");
            return;
        }
        print("Sensor %s data (stub)\n", argv[1]);
        // TODO: Show sensor data
    }, "Show sensor data");

    registerCommand("calib", [this](int argc, char** argv) {
        if (argc < 2) {
            print("Usage: calib [gyro|accel|mag]\n");
            return;
        }
        print("Calibrating %s (stub)\n", argv[1]);
        // TODO: Run calibration
    }, "Run calibration");

    registerCommand("motor", [this](int argc, char** argv) {
        if (argc < 2) {
            print("Usage: motor test <id> <throttle> | arm | disarm\n");
            return;
        }
        print("Motor command: %s (stub)\n", argv[1]);
        // TODO: Motor control
    }, "Motor control");

    registerCommand("pair", [this](int, char**) {
        print("Entering pairing mode (stub)\n");
        // TODO: Enter pairing mode
    }, "Enter pairing mode");

    registerCommand("reset", [this](int, char**) {
        print("Resetting system...\n");
        esp_restart();
    }, "Reset system");

    registerCommand("gain", [this](int argc, char** argv) {
        if (argc < 3) {
            print("Usage: gain <name> <value>\n");
            return;
        }
        print("Setting gain %s = %s (stub)\n", argv[1], argv[2]);
        // TODO: Set gain value
    }, "Set control gain");
}

}  // namespace stampfly
