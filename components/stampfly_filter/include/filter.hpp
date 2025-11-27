/**
 * @file filter.hpp
 * @brief Filter Library for Sensor Processing
 */
#pragma once
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace stampfly {
namespace filter {

/**
 * @brief Moving Average Filter
 */
template<typename T, size_t N>
class MovingAverage {
public:
    void update(T value) {
        buffer_[index_] = value;
        index_ = (index_ + 1) % N;
        if (count_ < N) count_++;
    }

    T get() const {
        if (count_ == 0) return T{};
        T sum = T{};
        for (size_t i = 0; i < count_; i++) {
            sum += buffer_[i];
        }
        return sum / static_cast<T>(count_);
    }

    void reset() {
        index_ = 0;
        count_ = 0;
    }

private:
    T buffer_[N] = {};
    size_t index_ = 0;
    size_t count_ = 0;
};

/**
 * @brief Median Filter
 */
template<typename T, size_t N>
class MedianFilter {
public:
    void update(T value) {
        buffer_[index_] = value;
        index_ = (index_ + 1) % N;
        if (count_ < N) count_++;
    }

    T get() const {
        if (count_ == 0) return T{};
        T sorted[N];
        for (size_t i = 0; i < count_; i++) {
            sorted[i] = buffer_[i];
        }
        std::sort(sorted, sorted + count_);
        return sorted[count_ / 2];
    }

    void reset() {
        index_ = 0;
        count_ = 0;
    }

private:
    T buffer_[N] = {};
    size_t index_ = 0;
    size_t count_ = 0;
};

/**
 * @brief First-order IIR Low Pass Filter
 */
class LowPassFilter {
public:
    LowPassFilter(float cutoff_freq, float sample_freq)
        : output_(0.0f) {
        float rc = 1.0f / (2.0f * 3.14159265f * cutoff_freq);
        float dt = 1.0f / sample_freq;
        alpha_ = dt / (rc + dt);
    }

    float update(float value) {
        output_ = alpha_ * value + (1.0f - alpha_) * output_;
        return output_;
    }

    void reset() {
        output_ = 0.0f;
    }

    float get() const { return output_; }

private:
    float alpha_;
    float output_;
};

/**
 * @brief Outlier Detector
 */
class OutlierDetector {
public:
    bool isOutlier(float value, float mean, float stddev, float n_sigma = 3.0f) const {
        return std::fabs(value - mean) > n_sigma * stddev;
    }

    bool isRateExceeded(float current, float previous, float max_rate, float dt) const {
        float rate = std::fabs(current - previous) / dt;
        return rate > max_rate;
    }

    bool isMahalanobisExceeded(float distance, float threshold) const {
        return distance > threshold;
    }
};

}  // namespace filter
}  // namespace stampfly
