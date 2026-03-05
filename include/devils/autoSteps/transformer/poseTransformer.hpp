#pragma once

#include "../../geometry/pose.hpp"

namespace devils
{
    /**
     * Base class for transforming poses.
     * Used to reuse autonomous steps across different alliance orientations.
     */
    struct PoseTransformer
    {
        virtual ~PoseTransformer() = default;
        /**
         * Transforms the pose to another location
         * @param pose - Input Pose
         * @returns Output pose
         */
        virtual Pose transform(Pose pose) = 0;
    };

    /// @brief Transforms the robot's pose by mirroring it over the x-axis
    class MirrorTransformX : public PoseTransformer
    {
        Pose transform(const Pose pose) override
        {
            return Pose(
                -pose.x,
                pose.y,
                M_PI - pose.rotation);
        }
    };

    /// @brief Transforms the robot's pose by mirroring it over the y-axis
    class MirrorTransformY : public PoseTransformer
    {
        Pose transform(const Pose pose) override
        {
            return Pose(
                pose.x,
                -pose.y,
                -pose.rotation);
        }
    };
}
