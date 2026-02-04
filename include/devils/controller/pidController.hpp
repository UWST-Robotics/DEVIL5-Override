#pragma once

#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents a feedback controller that uses a PID algorithm.
     */
    class PIDController
    {
    public:
        /// @brief The options for the PID controller.
        struct Options
        {
            /// @brief Proportional gain (p * error)
            float pGain = 0.0;

            /// @brief Integral gain (i * integral)
            float iGain = 0.0;

            /// @brief Derivative gain (d * derivative)
            float dGain = 0.0;
        };

        /**
         * Creates a new PID controller.
         * @param pGain The proportional gain of the controller (p * error)
         * @param iGain The integral gain of the controller (i * integral)
         * @param dGain The derivative gain of the controller (d * derivative)
         */
        PIDController(
            float pGain,
            float iGain,
            float dGain)
            : pGain(pGain),
              iGain(iGain),
              dGain(dGain)
        {
        }

        /**
         * Creates a new PID controller with the given options.
         * @param options The options for the PID controller.
         */
        PIDController(const Options& options)
            : pGain(options.pGain),
              iGain(options.iGain),
              dGain(options.dGain)
        {
        }

        /**
         * Resets the PID controller to its initial state.
         * Clears all error and integral values.
         * Should be called before starting a new control loop.
         */
        void reset()
        {
            currentError = 0;
            currentIntegral = 0;
            currentDerivative = 0;
            lastError = 0;
            lastUpdateTimestamp = pros::millis();
        }

        /**
         * Updates the PID controller with a new error value.
         * Should be called every control loop iteration.
         * @param error The current error value.
         * @return The current output value of the PID controller.
         */
        float update(const float error)
        {
            // Get Delta Time
            const auto dt = static_cast<float>(pros::millis() - lastUpdateTimestamp);
            lastUpdateTimestamp = pros::millis();

            // Update Error
            this->currentError = error;

            // Update Integral
            currentIntegral += error * dt;

            // Update Derivative
            if (dt > 0)
                currentDerivative = (error - lastError) / dt;
            lastError = error;

            // Return Value
            return getValue();
        }

        /**
         * Gets the last output value of the PID controller without updating it.
         * @return The last output value of the PID controller.
         */
        float getValue() const
        {
            const float p = pGain * currentError;
            const float i = iGain * currentIntegral;
            const float d = dGain * currentDerivative;

            return p + i + d;
        }

    private:
        // Feedback
        float currentError = 0;
        float currentIntegral = 0;
        float currentDerivative = 0;

        // Last Values
        float lastError = 0;
        uint32_t lastUpdateTimestamp = 0;

        // PID Variables
        float pGain;
        float iGain;
        float dGain;
    };
}
