#pragma once

#include "../geometry/pose.hpp"

namespace devils
{
    /**
     * Represents a pose with additional spline data.
     * Used in `SplinePath` to define control points.
     */
    struct SplinePose : Pose
    {
        SplinePose() = default;

        SplinePose(
            const float x,
            const float y,
            const float rotation,
            const float entryDelta,
            const float exitDelta)
            : Pose(x, y, rotation),
              entryDelta(entryDelta),
              exitDelta(exitDelta)
        {
        }

        /// @brief Distance of anchor point in inches from the start of the spline
        float entryDelta = 0;

        /// @brief Distance of anchor point in inches from the end of the spline
        float exitDelta = 0;

        /**
         * Gets the pose of the entry anchor point of the spline
         * @return The entry anchor pose
         */
        Pose getEntryAnchor() const
        {
            return {
                x + cosf(rotation) * -entryDelta,
                y + sinf(rotation) * -entryDelta,
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
                x + std::cos(rotation) * exitDelta,
                y + std::sin(rotation) * exitDelta,
                rotation
            };
        }
    };
}
