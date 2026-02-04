#pragma once

#include "poseTransform.h"

namespace devils
{
    /// @brief Transforms the robot's pose by rotating it 180 degrees
    class RotateTransform : public PoseTransform
    {
        Pose transform(Pose pose) override
        {
            return {
                -pose.x,
                -pose.y,
                static_cast<float>(M_PI) + pose.rotation
            };
        }
    };
}