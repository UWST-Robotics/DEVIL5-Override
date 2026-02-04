#pragma once

#include "../../chassis/chassisBase.hpp"
#include "../../odom/odomSource.hpp"
#include "../../path/path.hpp"
#include "autoDriveToStep.hpp"

namespace devils
{
    /**
     * Represents a step to follow a path using basic pure pursuit.
     */
    class AutoPurePursuitStep : public AutoDriveToStep
    {
    public:
        /**
         * Creates a new Pure Pursuit step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param path The path to follow.
         * @param options The options for the drive step.
         */
        AutoPurePursuitStep(
            ChassisBase& chassis,
            OdomSource& odomSource,
            Path* path,
            Options options = Options::defaultOptions)
            : AutoDriveToStep(chassis, odomSource, Pose(0, 0, 0), options),
              odomSource(odomSource),
              path(path)
        {
        }

    protected:
        void onStart() override
        {
            // Start the drive step
            AutoDriveToStep::onStart();
        }

        void onUpdate() override
        {
            // Get the current pose
            const Pose currentPose = odomSource.getPose();

            // Find the closest index on the path
            float closestIndex = 0;
            float closestDistance = std::numeric_limits<float>::infinity();
            for (float i = 0; i < path->getLength(); i += DELTA_T)
            {
                // Get the distance to the path at i
                Pose pathPose = path->getPoseAt(i);
                const float distance = currentPose.distanceTo(pathPose);

                // Check if this is the closest point
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestIndex = i;
                }
            }

            // Get the lookahead index
            float lookaheadIndex = closestIndex;
            while (lookaheadIndex < path->getLength() &&
                currentPose.distanceTo(path->getPoseAt(lookaheadIndex)) < LOOKAHEAD_DIST)
            {
                lookaheadIndex += DELTA_T;
            }

            // Set the target pose
            const Pose lookaheadPose = path->getPoseAt(lookaheadIndex);
            targetPose = lookaheadPose;

            // Update the drive step
            AutoDriveToStep::onUpdate();
        }

        void onStop() override
        {
            // Stop the drive step
            AutoDriveToStep::onStop();
        }

        bool checkFinished() override
        {
            // Check against the target pose, not the lookahead pose
            targetPose = path->getPoseAt(path->getLength() - 1);

            return AutoDriveToStep::checkFinished();
        }

        static constexpr float DELTA_T = 0.01; // indices
        static constexpr float LOOKAHEAD_DIST = 8.0; // inches

        OdomSource& odomSource;
        Path* path;
    };
}
