#pragma once

#include <utility>
#include <vector>
#include "devils/devilLib/path/splinePoseHolonomic.hpp"
#include "path.hpp"
#include "../geometry/lerp.hpp"

namespace devils
{
    /**
     * Cubic spline interpolated path
     */
    class SplinePathHolonomic : public Path
    {
    public:
        /// @brief The list of poses to interpolate between
        std::vector<SplinePoseHolonomic> poses;

        /**
         * Creates a new instance of a spline path using a list of SplinePoses.
         * @param poses The list of poses to interpolate between
         * @return The spline path
         */
        SplinePathHolonomic(
            std::vector<SplinePoseHolonomic> poses)
            : poses(std::move(poses))
        {
        }

        /**
         * Makes a simple 2 point arc path.
         * @param from The starting pose
         * @param to The ending pose
         * @param delta Distance of the entry and exit anchor points in inches
         * @return The spline path
         */
        static SplinePathHolonomic makeSpline(
            const Pose& from,
            const Pose& to,
            float anchorPointDist = 6.0,
            float entryPointAngle = 0.0f,
            float exitPointAngle = 0.0f)
        {
        
            std::vector<SplinePoseHolonomic> poses;
            poses.emplace_back(from.x, from.y, from.rotation, anchorPointDist, entryPointAngle);
            poses.emplace_back(to.x, to.y, to.rotation, anchorPointDist, exitPointAngle);
            return {poses};
        }

        /**
         * Gets a pose at a specific index along the path.
         * @param index The index to get the pose at. Interpolate between indices.
         * @return The pose at the index
         */
        Pose getPoseAt(const float index) override
        {
            // Check OOB
            if (index <= 0)
                return static_cast<Pose>(poses.front());
            if (index >= poses.size() - 1)
                return static_cast<Pose>(poses.back());

            // Get the two poses to interpolate between
            const int prevIndex = static_cast<int>(index);
            const int nextIndex = prevIndex + 1;
            const SplinePoseHolonomic prevPose = poses[prevIndex];
            const SplinePoseHolonomic nextPose = poses[nextIndex];
            const Pose prevAnchor = prevPose.getExitAnchor();
            const Pose nextAnchor = nextPose.getEntryAnchor();

            // Calculate dt between the two poses
            // This is the percentage of the way between the two poses
            const float dt = index - static_cast<float>(prevIndex);

            // Interpolate between the two poses
            // using cubic interpolation
            Pose interpolatedPose = Lerp::cubicPoints(
                prevPose,
                prevAnchor,
                nextAnchor,
                nextPose,
                dt);

            return interpolatedPose;
        }

        /**
         * Gets the length of the path
         * @return The length of the path in control points
         */
        float getLength() override
        {
            return static_cast<float>(poses.size());
        }

    private:
        /// @brief Step size used to calculate pose rotation
        static constexpr float DELTA_INDEX = 0.0001;
    };
}
