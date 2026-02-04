#pragma once
#include "pose.hpp"
#include "units.hpp"
#include <cmath>

namespace devils
{
    struct Lerp
    {
        Lerp() = delete;

        /**
         * Linearly interpolates a value from a to b.
         * @param a The minimum value.
         * @param b The maximum value.
         * @param t The ratio between a and b. Values between 0 and 1.
         * @return The interpolated value.
         */
        static float lerp(
            const float a,
            const float b,
            const float t)
        {
            return a + (b - a) * t;
        }

        /**
         * Linearly interpolates a rotational value from a to b.
         * @param a The minimum value in radians.
         * @param b The maximum value in radians.
         * @param t The ratio between a and b. Values between 0 and 1.
         */
        static float rotation(
            const float a,
            const float b,
            const float t)
        {
            const auto aMod = fmodf(a, 2.0f * M_PIF);
            const auto bMod = fmodf(b, 2.0f * M_PIF);
            const auto diff = std::abs(aMod - bMod);
            if (diff > M_PIF)
            {
                if (aMod > bMod)
                    return Units::normalizeRadians(lerp(aMod, bMod + 2.0f * M_PIF, t));

                return Units::normalizeRadians(lerp(aMod + 2.0f * M_PIF, bMod, t));
            }
            return Units::normalizeRadians(lerp(aMod, bMod, t));
        }

        /**
         * Linearly interpolates a point from a to b.
         * @param a The minimum point.
         * @param b The maximum point.
         * @param t The ratio between a and b. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose linearPoints(
            const Pose& a,
            const Pose& b,
            const float t)
        {
            return {
                lerp(a.x, b.x, t),
                lerp(a.y, b.y, t),
                rotation(a.rotation, b.rotation, t)
            };
        }

        /**
         * Quadratically interpolates a point from a to b to c.
         * @param a The minimum point.
         * @param b The middle point.
         * @param c The maximum point.
         * @param t The ratio between a and c. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose quadraticPoints(
            const Pose& a,
            const Pose& b,
            const Pose& c,
            const float t)
        {
            const Pose ab = linearPoints(a, b, t);
            const Pose bc = linearPoints(b, c, t);
            return linearPoints(ab, bc, t);
        }

        /**
         * Cubically interpolates a point from a to b to c to d.
         * @param a The minimum point.
         * @param b The middle point.
         * @param c The middle point.
         * @param d The maximum point.
         * @param t The ratio between a and d. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose cubicPoints(
            const Pose& a,
            const Pose& b,
            const Pose& c,
            const Pose& d,
            const float t)
        {
            const Pose abc = quadraticPoints(a, b, c, t);
            const Pose bcd = quadraticPoints(b, c, d, t);
            return linearPoints(abc, bcd, t);
        }
    };
}
