#pragma once
#include <string>
#include <cmath>

namespace devils
{
    /**
     * Represents various curve values to apply to joystick inputs.
     */
    struct JoystickCurve
    {
        // Prevent instantiation
        JoystickCurve() = delete;

        /**
         * Applies a deadzone to a joystick input.
         * @param deadzone The deadzone of the joystick.
         * @param val The value of the joystick. Must be between -1 and 1.
         * @return Value between deadzone and 1. Negative values are preserved.
         */
        static float deadzone(
            float deadzone,
            float val)
        {
            // Check if the value is within the deadzone
            const float absVal = std::abs(val);
            if (absVal < deadzone)
                return 0;
            if (absVal > 1)
                return std::copysign(1.0f, val);

            // Correct the value so it starts at 0 instead of deadzone
            float correctedVal = (absVal - deadzone) / (1 - deadzone);

            // Apply the sign back to the value
            return std::copysign(correctedVal, val);
        }

        /**
         * Applies an exponential curve to a joystick input.
         * @param val The value of the joystick. Must be between -1 and 1.
         * @param power The power of the curve.
         */
        static float pow(float val, float power)
        {
            const float absVal = std::abs(val);
            const float exponent = std::pow(absVal, power);
            return std::copysign(exponent, val);
        }

        /**
         * Lerp that maintains the sign of the value.
         * @param min The minimum value.
         * @param max The maximum value.
         * @param val The value to lerp.
         * @return The lerped value. If val is negative, the result will be negative.
         */
        static float lerp(float min, float max, float val)
        {
            const float absVal = std::fabs(val);
            const float lerpVal = std::lerp(min, max, absVal);
            return std::copysign(lerpVal, val);
        }

        /**
         * Applies a curve to a joystick input.
         * @param val The value of the joystick. Must be between -1 and 1.
         * @param power The power of the curve.
         * @param deadzone The deadzone of the joystick.
         * @param min The minimum output value
         * @param max The maximum output value
         */
        static float curve(
            const float val,
            const float power,
            const float deadzone,
            const float min = 0.0,
            const float max = 1.0)
        {
            // Deadzone
            float newVal = JoystickCurve::deadzone(deadzone, val);

            // Avoid remaining calculations if the value is 0
            if (newVal == 0)
                return 0;

            // Apply curve
            newVal = pow(newVal, power);

            // Lerp
            return lerp(min, max, newVal);
        }

        /**
         * Combines two joystick values. Chooses the value with the highest magnitude.
         * @param valueA The first value.
         * @param valueB The second value.
         * @return The combined value.
         */
        static float combine(
            const float valueA,
            const float valueB)
        {
            if (std::fabs(valueA) > std::fabs(valueB))
                return valueA;
            return valueB;
        }
    };
}
