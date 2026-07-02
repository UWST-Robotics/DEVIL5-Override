#pragma once

#include <utility>
#include <vector>
#include "path.hpp"
#include "splinePose.hpp"
#include "../geometry/lerp.hpp"

namespace devils
{
    /**
     * Cubic spline interpolated path
     */
    class SplinePath : public Path
    {
    public:
        /**
         * Creates a new instance of a spline path using a list of SplinePoses.
         * @param poses The list of poses to interpolate between
         * @param isReversed True if the path is reversed
         * @return The spline path
         */
        SplinePath(
            std::vector<SplinePose> poses,
            const bool isReversed = false)
            : isReversed(isReversed),
              poses(std::move(poses))
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
        static SplinePath makeArc(
            const Pose& from,
            const Pose& to,
            float delta = 6.0,
            bool isReversed = false)
        {
            if (isReversed)
                delta *= -1;

            std::vector<SplinePose> poses;
            poses.emplace_back(from.x, from.y, from.rotation, delta, delta);
            poses.emplace_back(to.x, to.y, to.rotation, delta, delta);
            return {poses, isReversed};
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
            const SplinePose prevPose = poses[prevIndex];
            const SplinePose nextPose = poses[nextIndex];
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

            const Pose interpolatedPosePrime = Lerp::cubicPoints(
                prevPose,
                prevAnchor,
                nextAnchor,
                nextPose,
                dt + DELTA_INDEX);

            // Calculate rotation
            // This is the instantaneous derivative of the pose at the given index
            const float rotation = std::atan2(
                interpolatedPosePrime.y - interpolatedPose.y,
                interpolatedPosePrime.x - interpolatedPose.x);
            interpolatedPose.rotation = rotation;

            // Reverse the rotation if the path is reversed
            if (isReversed)
                interpolatedPose.rotation = Units::normalizeRadians(interpolatedPose.rotation + M_PIF);

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

        /// @brief True if the path is reversed
        bool isReversed = false;

        /// @brief The list of poses to interpolate between
        std::vector<SplinePose> poses;
    };
}
