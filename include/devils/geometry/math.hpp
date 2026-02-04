#pragma once

#include <cmath>
#include <algorithm>
#include "vector2.hpp"

#ifndef M_PIF
#define M_PIF 3.141592653589793238462643383279502884e+00F
#endif

namespace devils
{
    struct Math
    {
        /**
         * Returns the minimum of two values by magnitude.
         * @param valueA The first value.
         * @param valueB The second value.
         * @return The value with the minimum magnitude.
         */
        static float minMagnitude(
            const float valueA,
            const float valueB)
        {
            if (std::abs(valueA) < std::abs(valueB))
                return valueA;
            return valueB;
        }

        /**
         * Returns the maximum of two values by magnitude.
         * @param valueA The first value.
         * @param valueB The second value.
         * @return The value with the maximum magnitude.
         */
        static float maxMagnitude(
            float valueA,
            float valueB)
        {
            if (std::abs(valueA) > std::abs(valueB))
                return valueA;
            return valueB;
        }

        /**
         * Interpolates a value along a sigmoid curve.
         * @param x The input value.
         * @param kCurve The curve factor. Must be greater than 1.
         * @return The value along the curve.
         */
        static float sigmoidCurve(
            const float x,
            const float kCurve)
        {
            // Avoid division by zero
            if (x == 1)
                return 1;

            // Calculate the curve
            const float hyperbolic = std::pow(x / (1 - x), kCurve);
            return 1 - 1 / (1 + hyperbolic);
        }

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
            if (value >= 0)
                return std::clamp(value, min, max);
            return std::clamp(value, -max, -min);
        }

        /**
         * Calculates the difference between two radian angles.
         * @param a The first angle in radians.
         * @param b The second angle in radians.
         * @return The minimum difference between the two angles in radians.
         */
        static float angleDiff(float a, float b)
        {
            float dist = a - b;
            dist = signedMod(dist + M_PIF, 2 * M_PIF) - M_PIF;
            return dist;
        }

        /**
         * Calculates the difference between two degree angles.
         * @param a The first angle in degrees.
         * @param b The second angle in degrees.
         * @return The minimum difference between the two angles in degrees.
         */
        static float angleDiffDeg(float a, float b)
        {
            float dist = a - b;
            dist = signedMod(dist + 180, 360) - 180;
            return dist;
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
    };
}
