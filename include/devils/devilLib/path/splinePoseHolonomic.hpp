#pragma once

#include "../geometry/pose.hpp"

namespace devils
{
    /**
     * Represents a pose with additional spline data.
     * Used in `SplinePath` to define control points.
     */
    struct SplinePoseHolonomic : Pose
    {
        SplinePoseHolonomic() = default;

        SplinePoseHolonomic(
            const float x,
            const float y,
            const float rotation,
            const float anchorDelta,
            const float anchorRotation)
            : Pose(x, y, rotation),
              anchorDelta(anchorDelta),
              anchorRotation(anchorRotation)
        {
        }

        /// @brief Distance of anchor point in inches from the start of the spline
        float anchorDelta = 6.0;
        float anchorRotation = 0.0;

        /**
         * Gets the pose of the entry anchor point of the spline
         * @return The entry anchor pose
         */
        Pose getEntryAnchor() const
        {
            return {
                x + cosf(anchorRotation) * -anchorDelta,
                y + sinf(anchorRotation) * -anchorDelta,
                rotation
            };
        }

        /**
         * Gets the pose of the exit anchor point of the spline
         * @return The exit anchor pose
         */
        Pose getExitAnchor() const
        {
            return {
                x + std::cos(anchorRotation) * anchorDelta,
                y + std::sin(anchorRotation) * anchorDelta,
                rotation
            };
        }
    };
}
