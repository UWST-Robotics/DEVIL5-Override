#pragma once

#include "../geometry/units.hpp"
#include "../odom/odomSource.hpp"
#include "../chassis/chassisBase.hpp"
#include "../trajectory/trajectoryGenerator.hpp"
#include "../path/splinePath.hpp"
#include "./steps/rotateMotionProfileStep.hpp"
#include "./steps/driveRAMSETEStep.hpp"

namespace devils
{
    /**
     * A builder for creating closed-loop autonomous routines w/ a chassis and odometry source.
     * Used to quickly create autonomous routines with a fluent API.
     */
    class AutoBuilder
    {
    public:
        /**
         * Creates a new autonomous builder with the given chassis and odometry source.
         * @param chassis The chassis to control
         * @param odom The odometry source to use
         */
        AutoBuilder(
            ChassisBase& chassis,
            OdomSource& odom)
            : chassis(chassis),
              odom(odom)
        {
        }

        /**
         * Shorthand to drive to a specific location on the field.
         * Uses 'AutoRamseteStep' by default, but can be changed to use other control methods in the future.
         * @param pose - The pose to drive to
         * @param endingVelocity - The desired velocity at the end of the trajectory in inches per second. Defaults to 0 for a full stop.
         * @returns A pointer to the created step
         */
        AutoStepPtr driveTo(
            const Pose& pose,
            const float endingVelocity = 0.0f) const
        {
            // TODO: Auto switch between `AutoRamseteStep` and `AutoHolonomicDriveStep` depending on the chassis type (holonomic or not).
            const auto trajectory = generateTrajectoryToPose(pose, endingVelocity);
            return std::make_shared<DriveRAMSETEStep>(chassis, odom, trajectory);
        }

        /**
         * Rotates the robot a given amount
         * @param distance The distance to rotate in degrees
         * @param options The options for the drive step
         * @returns A pointer to the created step
         */
        AutoStepPtr rotate(
            const float distance,
            const RotateMotionProfileStep::Options& options = RotateMotionProfileStep::Options::defaultOptions) const
        {
            const auto currentRotation = odom.getPose().rotation;
            const auto targetRotation = currentRotation + Units::degToRad(distance);
            return std::make_shared<RotateMotionProfileStep>(
                chassis,
                odom,
                targetRotation,
                options);
        }

        /**
         * Rotates the robot to a given heading
         * @param heading The heading to rotate to in degrees
         * @param options The options for the drive step
         * @returns A pointer to the created step
         */
        AutoStepPtr rotateTo(
            const float heading,
            RotateMotionProfileStep::Options options = RotateMotionProfileStep::Options::defaultOptions) const
        {
            return std::make_shared<RotateMotionProfileStep>(
                chassis,
                odom,
                Units::degToRad(heading),
                options);
        }

    protected:
        /**
         * Generates a trajectory to a given pose using the current pose as the starting point.
         * @param targetPose - The target pose to generate the trajectory to
         * @param endingVelocity - The desired velocity at the end of the trajectory in inches per second. Defaults to 0 for a full stop.
         * @return A shared pointer to the generated trajectory
         */
        std::shared_ptr<Trajectory> generateTrajectoryToPose(
            const Pose& targetPose,
            const float endingVelocity = 0.0f) const
        {
            // Check if the targetPose is behind the currentPose
            const auto currentPose = odom.getPose();
            const auto isReversed = targetPose.isBehind(currentPose);

            // Calculate starting velocity based on current velocity and its direction relative to the target pose
            const auto currentVelocity = odom.getVelocity();
            const auto deltaPose = targetPose - currentPose;
            const auto dotProduct = deltaPose.x * currentVelocity.x + deltaPose.y * currentVelocity.y;
            const auto velocityDirection = (dotProduct >= 0) ? 1.0f : 1.0f;
            const auto startingVelocity = velocityDirection * currentVelocity.magnitude();
            Logger::info("Starting Velocity : " + std::to_string(startingVelocity) + " in/s");

            // Generate Path
            const auto distance = currentPose.distanceTo(targetPose);
            auto path = SplinePath::makeArc(
                currentPose,
                targetPose,
                distance * 0.25f,
                isReversed);

            // Generate Trajectory
            const auto generator = TrajectoryGenerator({
                .startingVelocity = startingVelocity,
                .endingVelocity = endingVelocity
            });
            const auto trajectory = generator.calc(path);
            return trajectory;
        }

        ChassisBase& chassis;
        OdomSource& odom;
    };
}
