/**
 * @file eskf.hpp
 * @brief ESKF Position/Attitude Estimator (Placeholder)
 */
#pragma once
#include "esp_err.h"

namespace stampfly {

class ESKF {
public:
    esp_err_t init();
    void predict(float dt);
    void updateIMU(float ax, float ay, float az, float gx, float gy, float gz);
    void updateMag(float mx, float my, float mz);
    void updateBaro(float altitude);
    void updateOptFlow(float vx, float vy);
    void updateToF(float distance);
};

}  // namespace stampfly
