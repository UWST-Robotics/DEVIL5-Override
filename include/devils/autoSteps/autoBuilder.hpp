#pragma once

#include "../geometry/units.hpp"
#include "../odom/odomSource.hpp"
#include "../chassis/chassisBase.hpp"
#include "../trajectory/trajectoryConstraints.hpp"
#include "../trajectory/trajectoryGenerator.hpp"
#include "../path/splinePath.hpp"
#include "./steps/autoJumpToStep.hpp"
#include "./steps/autoDriveStep.hpp"
#include "./steps/autoDriveToStep.hpp"
#include "./steps/autoDriveTimeStep.hpp"
#include "./steps/autoRotateToStep.hpp"
#include "./steps/autoTimeoutStep.hpp"
#include "./steps/autoPauseStep.hpp"
#include "./steps/autoPurePursuitStep.hpp"
#include "./steps/autoBoomerangStep.hpp"
#include "./steps/autoRamseteStep.hpp"
#include "./transformer/poseTransform.h"

namespace devils
{
    class AutoBuilder
    {
    public:
        /// @brief The options passed in by the user in autoBuilder
        struct TrajectoryOptions
        {
            /// @brief If true, the robot will drive in reverse. Otherwise, it will drive forward.
            bool isReversed = false;

            /// @brief If true, the robot will start from the odometry pose. Otherwise, it will start from the theoretical current pose.
            bool startFromOdomPose = false;

            /// @brief The strength of the bezier curve (inches). Typically about 1/3 the path length.
            double strength = 2.0;

            /// @brief The expected velocity at the end of the trajectory (inches per second)
            double finalVelocity = 0;

            /// @brief The constraints for the trajectory
            TrajectoryConstraints constraints = {56.0f, 64.0f};

            /// @brief The options for the drive step
            AutoRamseteStep::Options driveOptions = AutoRamseteStep::Options::defaultOptions;
        };

        /**
         * A builder for creating closed-loop autonomous routines w/ a chassis and odometry source.
         * Used to quickly create autonomous routines with a fluent API.
         * @param chassis The chassis to control
         * @param odom The odometry source to use
         */
        AutoBuilder(
            ChassisBase &chassis,
            OdomSource &odom)
            : chassis(chassis),
              odom(odom)
        {
        }

        /**
         * Sets the pose of the robot
         * @param pose The pose to set the robot to
         */
        void setPose(Pose pose)
        {
            // Set the current pose
            this->pose = pose;

            // Transform the pose
            Pose transformedPose = tryTransformPose(pose);

            // Return a new `AutoJumpToStep` with the given pose
            this->odom.setPose(transformedPose);
        }

        /**
         * Sets the pose of the robot
         * @param x The x position to set the robot to in inches
         * @param y The y position to set the robot to in inches
         * @param rotation The rotation to set the robot to in degrees
         */
        void setPose(double x, double y, double rotation)
        {
            // Create a new pose
            Pose pose = Pose(x, y, Units::degToRad(rotation));

            // Return a new `AutoJumpToStep` with the given pose
            setPose(pose);
        }

        /**
         * Pauses the autonomous routine for a given duration
         * @param duration The duration to pause in milliseconds
         * @returns A pointer to the created step
         */
        AutoStepPtr pause(uint32_t duration)
        {
            velocity = 0.0;
            return std::make_shared<AutoPauseStep>(duration);
        }

        /**
         * Drives the robot to a given pose using a bezier curve, trajectory generation, and ramsete control.
         * @param x The x position to drive to in inches
         * @param y The y position to drive to in inches
         * @param rotation The rotation to drive to in degrees
         * @param isReversed Whether to drive in reverse or not
         * @param finalVelocity The final velocity to drive at in inches per second. Speed is carried over from the previous step.
         * @param strength The strength of the bezier curve (inches)
         * @param constraints The constraints for the trajectory
         * @param startFromOdomPose Whether to start from the odometry pose or the theoretical current pose
         * @param options The options for the drive step
         * @returns A pointer to the created step
         */
        AutoStepPtr driveToTrajectory(
            double x,
            double y,
            double rotation,
            bool isReversed = false,
            double finalVelocity = 0,
            double strength = 2.0,
            TrajectoryConstraints constraints = {56, 64},
            bool startFromOdomPose = false,
            AutoRamseteStep::Options options = AutoRamseteStep::Options::defaultOptions)
        {
            // Create a new pose
            Pose targetPose = Pose(x, y, Units::degToRad(rotation));

            // Return a new `AutoRamseteStep` with the given pose
            return driveToTrajectoryPose(
                targetPose,
                isReversed,
                finalVelocity,
                strength,
                constraints,
                startFromOdomPose,
                options);
        }

        /**
         * Drives the robot to a given pose using a bezier curve, trajectory generation, and ramsete control.
         * @param x The x position to drive to in inches
         * @param y The y position to drive to in inches
         * @param rotation The rotation to drive to in degrees
         * @param options The options for the drive step
         */
        AutoStepPtr driveToTrajectory(
            double x,
            double y,
            double rotation,
            TrajectoryOptions options)
        {
            return driveToTrajectory(
                x,
                y,
                rotation,
                options.isReversed,
                options.finalVelocity,
                options.strength,
                options.constraints,
                options.startFromOdomPose,
                options.driveOptions);
        }

        /**
         * Drives the robot to a given pose using a bezier curve, trajectory generation, and ramsete control.
         * @param pose The pose to drive to
         * @param isReversed Whether to drive in reverse or not
         * @param finalVelocity The final velocity to drive at in inches per second. Speed is carried over from the previous step.
         * @param strength The strength of the bezier curve (inches)
         * @param constraints The constraints for the trajectory
         * @param startFromOdomPose Whether to start from the odometry pose or the theoretical current pose
         * @param options The options for the drive step
         * @returns A pointer to the created step
         */
        AutoStepPtr driveToTrajectoryPose(
            Pose pose,
            bool isReversed = false,
            double finalVelocity = 0,
            double strength = 10.0,
            TrajectoryConstraints constraints = {48, 64},
            bool startFromOdomPose = false,
            AutoRamseteStep::Options options = AutoRamseteStep::Options::defaultOptions)
        {
            // Transform the pose
            Pose fromPose = startFromOdomPose ? tryTransformPose(odom.getPose()) : tryTransformPose(this->pose);
            Pose toPose = tryTransformPose(pose);

            // Create a new path
            SplinePath path = SplinePath::makeArc(fromPose, toPose, strength, isReversed);

            // Flip final velocity if the path is reversed
            if (isReversed)
                finalVelocity *= -1;

            // Generate Trajectory
            auto trajectoryGenerator = TrajectoryGenerator(
                constraints,
                TrajectoryGenerator::PathInfo{velocity, finalVelocity});
            auto trajectory = trajectoryGenerator.calc(path);

            // Set the current pose
            this->pose = pose;
            velocity = finalVelocity;

            // Make a new `AutoRamseteStep` with the given trajectory
            return std::make_shared<AutoRamseteStep>(chassis, odom, trajectory, options);
        }

        /**
         * Rotates the robot a given amount
         * @param distance The distance to rotate in degrees
         * @param timeout The timeout in milliseconds
         * @param options The options for the drive step
         * @returns A pointer to the created step
         */
        AutoStepPtr rotate(
            double distance,
            uint32_t timeout = 2000,
            AutoRotateToStep::Options options = AutoRotateToStep::Options::defaultOptions)
        {
            // Calculate the new heading (in degrees)
            double newHeading = Units::radToDeg(pose.rotation) + distance;

            // Return a new `AutoRotateToStep` with the given heading
            return rotateTo(newHeading, timeout, options);
        }

        /**
         * Drives the robot a given distance in closed loop
         * @param distance The distance to drive in inches
         * @param isReversed Whether to drive in reverse or not
         * @param finalVelocity The final velocity to drive at in inches per second. Speed is carried over from the previous step.
         * @param strength The strength of the bezier curve (inches)
         * @param constraints The constraints for the trajectory
         * @param startFromOdomPose Whether to start from the odometry pose or the theoretical current pose
         * @param options The options for the drive step
         */
        AutoStepPtr driveTrajectory(
            double distance,
            bool isReversed = false,
            double finalVelocity = 0,
            double strength = 10.0,
            TrajectoryConstraints constraints = {48, 64},
            bool startFromOdomPose = false,
            AutoRamseteStep::Options options = AutoRamseteStep::Options::defaultOptions)
        {
            // Create a new pose
            Pose targetPose = Pose(pose.x + distance * std::cos(pose.rotation),
                                   pose.y + distance * std::sin(pose.rotation),
                                   pose.rotation);

            // Return a new `AutoDriveStep` with the given pose
            return driveToTrajectoryPose(
                targetPose,
                isReversed,
                finalVelocity,
                strength,
                constraints,
                startFromOdomPose,
                options);
        }

        /**
         * Rotates the robot to a given heading
         * @param heading The heading to rotate to in degrees
         * @param timeout The timeout in milliseconds
         * @returns A pointer to the created step
         */
        AutoStepPtr rotateTo(
            double heading,
            uint32_t timeout = 2000,
            AutoRotateToStep::Options options = AutoRotateToStep::Options::defaultOptions)
        {
            // Convert & apply the heading to the current pose
            pose.rotation = Units::degToRad(heading);
            velocity = 0.0;

            // Transform the pose
            Pose transformedPose = tryTransformPose(pose);

            // Return a new `AutoRotateToStep` with the given heading
            return std::make_shared<AutoTimeoutStep>(std::make_shared<AutoRotateToStep>(chassis, odom, transformedPose.rotation, options), timeout);
        }

        /**
         * Uses a pose transformation when building autonomous
         * @param transformer - The transformation to apply
         */
        void useTransformer(std::unique_ptr<PoseTransform> transformer)
        {
            this->transformer = std::move(transformer);
        }

    protected:
        /**
         * Tries to transform a pose using the assigned transformer, if any
         * @param pose - The pose to transform
         * @returns The transformed pose
         */
        Pose tryTransformPose(Pose pose)
        {
            // If a transformer is assigned, transform the pose
            if (transformer)
                return transformer->transform(pose);

            // Otherwise, return the original pose
            return pose;
        }

    private:
        /// @brief The current robot pose (pre-transform)
        Pose pose;

        /// @brief The current velocity of the robot in inches per second
        double velocity = 0.0;

        /// @brief The active transformer used to transform poses
        std::unique_ptr<PoseTransform> transformer = nullptr;

        // Input references
        ChassisBase &chassis;
        OdomSource &odom;
    };
}