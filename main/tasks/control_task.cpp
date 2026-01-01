/**
 * @file control_task.cpp
 * @brief 制御タスク (400Hz) - 姿勢制御（ユーザー実装用スタブ）
 */

#include "tasks_common.hpp"

static const char* TAG = "ControlTask";

using namespace config;
using namespace globals;

/**
 * @brief Control Task - 400Hz (2.5ms period)
 *
 * This task handles flight control (attitude/position control, motor mixing).
 * It runs at 400Hz, synchronized with IMU updates via semaphore.
 *
 * ============================================================================
 * STUB IMPLEMENTATION - Replace with your flight control code
 * ============================================================================
 *
 * Typical flight control loop:
 * 1. Read current state (attitude, position, velocity) from StampFlyState
 * 2. Read control inputs (throttle, roll, pitch, yaw commands)
 * 3. Compute control outputs using PID or other control algorithms
 * 4. Apply motor mixing for X-quad configuration
 * 5. Send PWM commands to motors
 *
 * See docs/developer_guide.md for detailed PID control example.
 */
void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started (400Hz via semaphore)");

    auto& state = stampfly::StampFlyState::getInstance();

    // ========================================================================
    // Motor Layout (X-quad configuration, viewed from above)
    // ========================================================================
    //
    //               Front
    //          FL (M4)   FR (M1)
    //             ╲   ▲   ╱
    //              ╲  │  ╱
    //               ╲ │ ╱
    //                ╲│╱
    //                 ╳         ← Center of drone
    //                ╱│╲
    //               ╱ │ ╲
    //              ╱  │  ╲
    //             ╱   │   ╲
    //          RL (M3)    RR (M2)
    //                Rear
    //
    // Motor rotation:
    //   M1 (FR): CCW (Counter-Clockwise)
    //   M2 (RR): CW  (Clockwise)
    //   M3 (RL): CCW (Counter-Clockwise)
    //   M4 (FL): CW  (Clockwise)
    //
    // ========================================================================

    while (true) {
        // Wait for control semaphore (given by IMU task after ESKF update)
        if (xSemaphoreTake(g_control_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Get current flight state
        stampfly::FlightState flight_state = state.getFlightState();

        // Only run control when ARMED or FLYING
        if (flight_state != stampfly::FlightState::ARMED &&
            flight_state != stampfly::FlightState::FLYING) {
            // Ensure motors are stopped when not armed
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_FR, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_RR, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_RL, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_FL, 0);
            continue;
        }

        // Get control input from controller
        // throttle: 0.0 ~ 1.0
        // roll, pitch, yaw: -1.0 ~ +1.0 (not used in this simple example)
        float throttle, roll, pitch, yaw;
        state.getControlInput(throttle, roll, pitch, yaw);

        // Simple throttle control: all motors receive same throttle value
        // TODO: Add attitude control (PID) and motor mixing for stable flight
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_FR, throttle);  // M1
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_RR, throttle);  // M2
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_RL, throttle);  // M3
        g_motor.setMotor(stampfly::MotorDriver::MOTOR_FL, throttle);  // M4
    }
}
