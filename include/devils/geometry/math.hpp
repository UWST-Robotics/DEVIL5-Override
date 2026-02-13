#pragma once

#include <cmath>
#include <algorithm>
#include "vector2.hpp"

#ifndef M_PIF
#define M_PIF 3.141592653589793238462643383279502884e+00F
#endif

namespace devils
{
    /**
     * Collection of utility math functions.
     */
    struct Math
    {
        // Delete constructor to prevent instantiation
        Math() = delete;
        
        /**
         * Modulus function that works with negative numbers.
         * For example, -1 % 3 = 2 and -1 % -3 = 1.
         * @param a The dividend.
         * @param b The divisor.
         * @return The remainder.
         */
        static float signedMod(
            const float a,
            const float b)
        {
            return a - b * std::floor(a / b);
        }

        /**
         * Clamps a value between a minimum and maximum.
         * Allows for a deadband around zero such that negative values are clamped to -min and positive values are clamped to +min.
         * @param value The value to clamp.
         * @param min The minimum value.
         * @param max The maximum value.
         * @return The clamped value.
         */
        static float deadbandClamp(
            const float value,
            const float min,
            const float max)
        {
            if (value > 0)
                return std::clamp(value, min, max);
            if (value < 0)
                return std::clamp(value, -max, -min);
            
            return 0;
        }

        /**
         * Calculates the end velocity over a specified distance given initial velocity and acceleration.
         * @param initialVelocity The initial velocity in units per second.
         * @param acceleration Constant acceleration in units per second squared.
         * @param distance The distance in units.
         */
        static float velocityOverDist(
            const float initialVelocity,
            const float acceleration,
            const float distance)
        {
            return sqrtf(
                initialVelocity * initialVelocity +
                2 * acceleration * distance);
        }

        /**
         * Calculates the acceleration over a specified distance given initial and final velocities.
         * @param initialVelocity The initial velocity in units per second.
         * @param finalVelocity The final velocity in units per second.
         * @param distance The distance in units.
         */
        static float accelOverDist(
            const float initialVelocity,
            const float finalVelocity,
            const float distance)
        {
            // (v_f^2 - v_i^2) / 2d
            return (finalVelocity * finalVelocity - initialVelocity * initialVelocity) / (2 * distance);
        }

        /**
         * Calculates the distance between b and c along the line defined by a and b.
         * @param a The first point on the line.
         * @param b The second point on the line. Also, the point to measure from.
         * @param c The point to measure to.
         * @return The distance between b and c along the line defined by a and b.
         */
        static float distanceOnLine(
            const Vector2& a,
            const Vector2& b,
            const Vector2& c)
        {
            const float numerator = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
            const float denominator = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            const float t = numerator / denominator;

            const float x = b.x - ((1 - t) * a.x + t * b.x);
            const float y = b.y - ((1 - t) * a.y + t * b.y);
            return sqrtf(x * x + y * y);
        }

        /**
         * Chooses the value with the largest magnitude from a list of values.
         * @param values - A list of values to find the largest magnitude of.
         * @return The value with the largest magnitude in the list.
         */
        static float largestMagnitude(const std::initializer_list<float>& values)
        {
            float largest = 0;
            for (const auto& value : values)
            {
                if (std::abs(value) > std::abs(largest))
                    largest = value;
            }
            return largest;
        }

        /**
         * Chooses the value with the smallest magnitude from a list of values.
         * @param values - A list of values to find the smallest magnitude of.
         * @return The value with the smallest magnitude in the list. 
         */
        static float smallestMagnitude(const std::initializer_list<float>& values)
        {
            float smallest = 0;
            for (const auto& value : values)
            {
                if (std::abs(value) < std::abs(smallest) || smallest == 0)
                    smallest = value;
            }
            return smallest;
        }
    };
}
