#pragma once
#include <string>
#include <cmath>
#include "math.hpp"

namespace devils
{
    /**
     * A class containing a variety of unit conversion utilities.
     */
    class Units
    {
    public:
        /**
         * Converts inches to meters.
         * @param inches The inches to convert.
         * @return The inches in meters.
         */
        static float inToMeters(const float inches)
        {
            return inches * 0.0254f;
        }

        /**
         * Converts meters to inches.
         * @param meters The meters to convert.
         * @return The meters in inches.
         */
        static float metersToIn(const float meters)
        {
            return meters / 0.0254f;
        }

        /**
         * Converts degrees to radians.
         * @param degrees The degrees to convert.
         * @return The degrees in radians.
         */
        static float degToRad(const float degrees)
        {
            return degrees * (M_PIF / 180.0f);
        }

        /**
         * Converts radians to degrees.
         * @param radians The radians to convert.
         */
        static float radToDeg(const float radians)
        {
            return radians * (180.0f / M_PIF);
        }
        
        /**
         * Converts centidegrees to radians.
         * @param centidegrees The centidegrees to convert.
         * @return The centidegrees in radians.
         */
        static float centidegToRad(const float centidegrees)
        {
            return centidegrees * (M_PIF / 18000.0f);
        }

        /**
         * Converts radians to centidegrees.
         * @param radians - The radians to convert.
         * @return The radians in centidegrees.
         */
        static float radToCentideg(const float radians)
        {
            return radians * (18000.0f / M_PIF);
        }

        /**
         * Converts centidegrees to degrees.
         * @param centidegrees The centidegrees to convert.
         * @return The centidegrees in degrees.
         */
        static float centidegToDeg(const float centidegrees)
        {
            return centidegrees / 100.0f;
        }

        /**
         * Converts degrees to centidegrees.
         * @param degrees The degrees to convert.
         * @return The degrees in centidegrees.
         */
        static float degToCentideg(const float degrees)
        {
            return degrees * 100.0f;
        }
        
        /**
         * Compares the difference of two radian angles.
         * @param radiansA The first angle in radians.
         * @param radiansB The second angle in radians.
         * @return The difference between the two angles, relative to `radiansB`. Can be [-PI, PI]
         */
        static float diffRad(const float radiansA, const float radiansB)
        {
            return std::atan2(
                std::sin(radiansA - radiansB),
                std::cos(radiansA - radiansB));
        }

        /**
         * Compares the difference of two degree angles.
         * @param degreesA - The first angle in degrees.
         * @param degreesB - The second angle in degrees.
         * @return The difference between the two angles, relative to `degreesB`. Can be [-180, 180]
         */
        static float diffDeg(const float degreesA, const float degreesB)
        {
            return radToDeg(diffRad(
                degToRad(degreesA),
                degToRad(degreesB)));
        }
        
        /**
         * Modulus function that also works with negative numbers.
         * @param a The number to mod.
         * @param b The modulus.
         * @return The result of the modulus. Always positive.
         */
        static float mod(const float a, const float b)
        {
            return a - std::floor(a / b) * b;
        }
        
        /**
         * Normalizes an angle in radians to be between 0 and 2 * PI.
         * @param radians The angle in radians to normalize.
         * @return The normalized angle in radians.
         */
        static float normalizeRadians(const float radians)
        {
            return mod(radians, 2.0f * M_PIF);
        }
    };
}
