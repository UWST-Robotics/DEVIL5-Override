#pragma once

#include "../autoStep.hpp"
#include "../../odom/odomSource.hpp"

namespace devils
{
    /**
     * Jumps the odometry state to a given pose. Usually ran at the start of an auto routine.
     */
    class AutoJumpToStep : public AutoStep
    {
    public:
        /**
         * Jumps the odometry state to a given pose. Usually ran at the start of an auto routine.
         * @param odom The odometry source to use.
         * @param x The x position to jump to in inches.
         * @param y The y position to jump to in inches.
         * @param heading The heading to jump to in radians.
         */
        AutoJumpToStep(
            OdomSource& odom,
            const float x,
            const float y,
            const float heading)
            : odom(odom),
              targetPose(x, y, heading)
        {
        }

        /**
         * Jumps the odometry state to a given pose. Usually ran at the start of an auto routine.
         * @param odom The odometry source to use.
         * @param pose The pose to jump to.
         */
        AutoJumpToStep(OdomSource& odom, const Pose& pose)
            : odom(odom),
              targetPose(pose)
        {
        }

    protected:
        void onStart() override
        {
            odom.setPose(targetPose);
        }

        bool checkFinished() override
        {
            return true;
        }

        // Params
        OdomSource& odom;
        Pose targetPose;
    };
}
