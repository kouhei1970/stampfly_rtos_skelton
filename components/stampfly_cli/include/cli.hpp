/**
 * @file cli.hpp
 * @brief CLI Console
 */
#pragma once
#include <functional>
#include "esp_err.h"

namespace stampfly {

class CLI {
public:
    static CLI& getInstance();

    esp_err_t init();
    void registerCommand(const char* name, const char* help,
                        std::function<void(int, char**)> handler);
    void process();
    void print(const char* fmt, ...);
    void printTeleplot(const char* name, float value);

private:
    CLI() = default;
};

}  // namespace stampfly
