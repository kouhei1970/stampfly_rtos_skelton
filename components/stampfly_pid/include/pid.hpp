/**
 * @file pid.hpp
 * @brief PID Controller with Incomplete Derivative and Anti-windup
 *
 * Features:
 * - Incomplete derivative (filtered derivative): Td·s / (η·Td·s + 1)
 * - Bilinear transform (Tustin) for discretization
 * - Back-calculation anti-windup
 * - Derivative-on-Measurement (D-on-M) option
 *
 * Transfer function:
 *   C(s) = Kp(1 + 1/(Ti·s) + Td·s/(η·Td·s + 1))
 *
 * @see docs/pid_control.md for theory and tuning guidelines
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace stampfly {

/**
 * @brief PID Controller Configuration
 */
struct PIDConfig {
    float Kp = 1.0f;            ///< Proportional gain
    float Ti = 1.0f;            ///< Integral time [s] (<=0 disables integral)
    float Td = 0.0f;            ///< Derivative time [s] (<=0 disables derivative)
    float eta = 0.1f;           ///< Incomplete derivative filter coefficient (0.1~0.2 typical)
    float Tt = 0.0f;            ///< Tracking time constant for anti-windup (0 = auto: sqrt(Ti*Td))
    float output_min = -1.0f;   ///< Output lower limit
    float output_max = 1.0f;    ///< Output upper limit
    bool derivative_on_measurement = true;  ///< true: D-on-M, false: D-on-E
};

/**
 * @brief PID Controller with Incomplete Derivative and Anti-windup
 *
 * Usage:
 * @code
 * PID pid;
 * PIDConfig config;
 * config.Kp = 0.5f;
 * config.Ti = 1.0f;
 * config.Td = 0.02f;
 * pid.init(config, 0.0025f);  // dt = 2.5ms
 *
 * // In control loop:
 * float output = pid.update(setpoint, measurement);
 * @endcode
 */
class PID {
public:
    PID() = default;

    /**
     * @brief Initialize PID controller
     * @param config PID configuration parameters
     * @param dt Sample time [s]
     */
    void init(const PIDConfig& config, float dt);

    /**
     * @brief Compute PID output
     * @param setpoint Target value
     * @param measurement Current measured value
     * @return Control output (clamped to [output_min, output_max])
     */
    float update(float setpoint, float measurement);

    /**
     * @brief Reset internal states (integral, derivative filter, previous values)
     */
    void reset();

    /**
     * @brief Set proportional gain (runtime adjustment)
     */
    void setKp(float kp) { Kp_ = kp; }

    /**
     * @brief Set integral time (runtime adjustment)
     * @param ti Integral time [s] (<=0 disables integral)
     */
    void setTi(float ti);

    /**
     * @brief Set derivative time (runtime adjustment)
     * @param td Derivative time [s] (<=0 disables derivative)
     */
    void setTd(float td);

    /**
     * @brief Set output limits (runtime adjustment)
     */
    void setOutputLimits(float min, float max) {
        output_min_ = min;
        output_max_ = max;
    }

    // Getters for debugging
    float getProportional() const { return P_; }
    float getIntegral() const { return I_; }
    float getDerivative() const { return D_; }
    float getError() const { return error_; }

    // Configuration getters
    float getKp() const { return Kp_; }
    float getTi() const { return Ti_; }
    float getTd() const { return Td_; }
    float getDt() const { return dt_; }

private:
    /**
     * @brief Update integral term with trapezoidal integration
     * @param error Current error
     * @return Integral term contribution
     */
    float updateIntegral(float error);

    /**
     * @brief Update derivative term with incomplete derivative filter
     * @param error Current error
     * @param measurement Current measurement (for D-on-M)
     * @return Derivative term contribution
     */
    float updateDerivative(float error, float measurement);

    /**
     * @brief Apply back-calculation anti-windup
     * @param output_unlimited Unlimited PID output
     * @param output_limited Limited (clamped) output
     */
    void applyAntiWindup(float output_unlimited, float output_limited);

    /**
     * @brief Clamp value to output limits
     */
    float clamp(float value) const {
        if (value > output_max_) return output_max_;
        if (value < output_min_) return output_min_;
        return value;
    }

    // Configuration
    float Kp_ = 1.0f;
    float Ti_ = 1.0f;
    float Td_ = 0.0f;
    float eta_ = 0.1f;
    float Tt_ = 1.0f;
    float output_min_ = -1.0f;
    float output_max_ = 1.0f;
    bool derivative_on_measurement_ = true;
    float dt_ = 0.0025f;

    // Precomputed coefficients for derivative filter
    // D[k] = -a * D[k-1] + b * (input[k] - input[k-1])
    float deriv_a_ = 0.0f;
    float deriv_b_ = 0.0f;

    // Internal states
    float integral_ = 0.0f;          ///< Integral accumulator
    float deriv_filtered_ = 0.0f;    ///< Filtered derivative state
    float prev_error_ = 0.0f;        ///< Previous error (for integral)
    float prev_deriv_input_ = 0.0f;  ///< Previous derivative input (error or -measurement)
    bool first_run_ = true;          ///< First update flag

    // Output components (for debugging)
    float P_ = 0.0f;
    float I_ = 0.0f;
    float D_ = 0.0f;
    float error_ = 0.0f;
};

}  // namespace stampfly
