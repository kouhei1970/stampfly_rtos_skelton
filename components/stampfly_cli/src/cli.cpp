/**
 * @file cli.cpp
 * @brief CLI Implementation (Placeholder)
 */
#include "cli.hpp"
#include "esp_log.h"
#include <cstdio>
#include <cstdarg>

static const char* TAG = "cli";

namespace stampfly {

CLI& CLI::getInstance() {
    static CLI instance;
    return instance;
}

esp_err_t CLI::init() {
    ESP_LOGI(TAG, "CLI init (placeholder)");
    // TODO: Implement USB CDC console
    return ESP_OK;
}

void CLI::registerCommand(const char* name, const char* help,
                         std::function<void(int, char**)> handler) {
    // TODO: Implement command registration
}

void CLI::process() {
    // TODO: Implement command processing
}

void CLI::print(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void CLI::printTeleplot(const char* name, float value) {
    printf(">%s:%.4f\n", name, value);
}

}  // namespace stampfly
