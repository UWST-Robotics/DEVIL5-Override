#pragma once
#include <cmath>
#include <algorithm>

namespace devils
{
    /**
     * Represents a feedforward controller for an elevator.
     */
    class ElevatorFeedforward
    {
    public:
        struct Options
        {
            /// @brief Amount of gravitational force applied to the elevator.
            float gravity = 0.0;

            /// @brief Amount of force required to overcome static friction.
            float staticFriction = 0.0;

            /// @brief Amount of voltage to apply a given velocity.
            float velocityGain = 0.0;

            /// @brief Amount of voltage to apply a given acceleration.
            float accelerationGain = 0.0;
        };

        ElevatorFeedforward(const Options& options)
            : options(options)
        {
        }

        /**
         * Updates the feedforward voltage.
         * @param velocity The desired velocity of the elevator.
         * @param acceleration The desired acceleration of the elevator.
         * @return The voltage to apply to the motor.
         */
        float update(
            const float velocity = 0,
            const float acceleration = 0) const
        {
            // V = output voltage
            // k_g = gravity
            // k_s = static friction
            // k_v = velocity gain
            // k_a = acceleration gain
            // V = k_g + k_s * sign(vel) + k_v * vel + k_a * acc

            float voltage = 0.0;
            voltage += options.gravity; // Gravity
            if (velocity != 0)
                voltage += options.staticFriction * std::copysign(1.0f, velocity); // Static friction
            voltage += options.velocityGain * velocity; // Velocity
            voltage += options.accelerationGain * acceleration; // Acceleration

            // Clamp the voltage to the range [-1, 1]
            voltage = std::clamp(voltage, -1.0f, 1.0f);

            return voltage;
        }

    private:
        Options options;
    };
}
