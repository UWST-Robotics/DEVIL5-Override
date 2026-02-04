#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../geometry/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "autoDriveToStep.hpp"

namespace devils
{
    /**
     * Drives the robot a specific distance w/ odometry feedback.
     */
    class AutoDriveStep : public AutoDriveToStep
    {
    public:
        /**
         * Drives the robot a specific distance w/ odometry feedback.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param distance The distance to drive in inches.
         * @param options The options for the drive step.
         */
        AutoDriveStep(
            ChassisBase& chassis,
            OdomSource& odomSource,
            float distance,
            Options options = Options::defaultOptions)
            : AutoDriveToStep(chassis, odomSource, Pose(0, 0, 0), options),
              distance(distance)
        {
        }

    protected:
        void onStart() override
        {
            // Calculate Target Pose
            const Pose startPose = odomSource.getPose();
            this->targetPose = Pose(
                startPose.x + distance * std::cos(startPose.rotation),
                startPose.y + distance * std::sin(startPose.rotation),
                startPose.rotation);

            // Do base step
            AutoDriveToStep::onStart();
        }

        // Drive Step Variables
        float distance = 0;
    };
}
