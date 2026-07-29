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
        std::vector<Pose> poses;
        /**
         * Creates a new instance of a linear path using a list of poses.
         * @param poses The list of poses to interpolate between
         */
        LinearPath(std::vector<Pose> poses)
            : poses(std::move(poses))
        {
        }

        /**
         * Makes a simple 2 point arc path.
         * @param from The starting pose
         * @param to The ending pose
         * @param delta Distance of the entry and exit anchor points in inches
         * @param isReversed True if the path is reversed
         * @return The spline path
         */
        static LinearPath makeLinearPath(
            const Pose& from,
            const Pose& to)
        {
            std::vector<Pose> poses;
            poses.emplace_back(from.x, from.y, from.rotation);
            poses.emplace_back(to.x, to.y, to.rotation);
            return {poses};
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
        //
    };
}
