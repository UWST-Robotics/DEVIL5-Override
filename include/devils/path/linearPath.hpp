#pragma once

#include <utility>
#include <vector>
#include "path.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/lerp.hpp"

namespace devils
{
    /**
     * Linearly interpolated path
     */
    class LinearPath : public Path
    {
    public:
        /**
         * Creates a new instance of a linear path using a list of poses.
         * @param poses The list of poses to interpolate between
         */
        LinearPath(std::vector<Pose> poses)
            : poses(std::move(poses))
        {
        }

        Pose getPoseAt(const float index) override
        {
            // Check OOB
            if (index <= 0)
                return poses.front();
            if (index >= poses.size() - 1)
                return poses.back();

            // Get the two poses to interpolate between
            const int intIndex = static_cast<int>(index);
            const Pose prevPose = poses[intIndex];
            const Pose nextPose = poses[intIndex + 1];

            // Interpolate between the two poses
            const float t = index - intIndex;
            return Lerp::linearPoints(prevPose, nextPose, t);
        }

        float getLength() override
        {
            return poses.size();
        }

    private:
        std::vector<Pose> poses;
    };
}
