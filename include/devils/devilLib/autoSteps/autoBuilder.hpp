#pragma once

#include "../geometry/units.hpp"
#include "../odom/odomSource.hpp"
#include "../chassis/chassisBase.hpp"
#include "../trajectory/trajectoryGenerator.hpp"
#include "../path/splinePath.hpp"
#include "../path/linearPath.hpp"
#include "./steps/rotateMotionProfileStep.hpp"
#include "./steps/rotateStep.hpp"
#include "./steps/driveRAMSETEStep.hpp"
#include "./steps/driveHolonomicStep.hpp"
#include "./transformer/poseTransformer.hpp"

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
            const auto transformedPose = tryTransformPose(pose);

            const auto trajectory = generateTrajectoryToPose(transformedPose, endingVelocity);
            // Auto switch between `AutoRamseteStep` and `AutoHolonomicDriveStep` depending on the chassis type (holonomic or not).
            if (chassis.isHolonomic()){
                return std::make_shared<DriveHolonomicStep>(chassis, odom, trajectory);
            } else {
                return std::make_shared<DriveRAMSETEStep>(chassis, odom, trajectory);
            }
        }

        AutoStepPtr driveToLinear(
            const Pose& pose,
            const float endingVelocity = 0.0f) const
        {
            const auto transformedPose = tryTransformPose(pose);

            const auto trajectory = generateTrajectoryToPoseLinear(transformedPose, endingVelocity);
            // Auto switch between `AutoRamseteStep` and `AutoHolonomicDriveStep` depending on the chassis type (holonomic or not).
            if (chassis.isHolonomic()){
                return std::make_shared<DriveHolonomicStep>(chassis, odom, trajectory);
            } else {
                return std::make_shared<DriveRAMSETEStep>(chassis, odom, trajectory);
            }
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
            // TODO: Transform pose here

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
            Logger::info("I running");
            const auto transformedPose = tryTransformPose(Pose(0, 0, Units::degToRad(heading)));

            return std::make_shared<RotateMotionProfileStep>(
                chassis,
                odom,
                transformedPose.rotation,
                options);
        }

        /**
         * Sets the robot's current pose to a given pose. Useful for resetting odometry or "teleporting" the robot during autonomous.
         * @param pose - The pose to set the robot to
         */
        void jumpTo(const Pose& pose) const
        {
            const auto transformedPose = tryTransformPose(pose);
            odom.setPose(transformedPose);
        }

        /**
         * Assigns a transformer to the auto builder to transform all poses used in the builder.
         * @param newTransformer - The new transformer to use. Can be `nullptr` to remove the current transformer.
         */
        void setPoseTransformer(std::unique_ptr<PoseTransformer> newTransformer)
        {
            transformer = std::move(newTransformer);
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
            //const auto currentVelocity = odom.getVelocity();
            const auto currentVelocity = Vector2(0,0);
            const auto deltaPose = targetPose - currentPose;
            const auto dotProduct = deltaPose.x * currentVelocity.x + deltaPose.y * currentVelocity.y;
            //const auto velocityDirection = (dotProduct >= 0) ? 1.0f : -1.0f;
            const auto velocityDirection = 1.0f;
            const auto startingVelocity = velocityDirection * currentVelocity.magnitude();
            Logger::info("Starting Velocity : " + std::to_string(startingVelocity) + " in/s");

            // Generate Path
            const auto distance = currentPose.distanceTo(targetPose);
            auto path = SplinePath::makeArc(
                currentPose,
                targetPose,
                distance * 0.25f,
                isReversed);

            std::string filestring = "";
            for (int i=0; i < path.poses.size(); i++){
                filestring += path.poses[i].toString();
                filestring +="\n";
            }
            SDCardHandler sdHandler;
            sdHandler.writeFile("spline.csv", filestring);

            // Generate Trajectory
            const auto generator = TrajectoryGenerator({
                .startingVelocity = startingVelocity,
                .endingVelocity = endingVelocity
            });
            const auto trajectory = generator.calc(path);
            return trajectory;
        }

        std::shared_ptr<Trajectory> generateTrajectoryToPoseLinear(
            const Pose& targetPose,
            const float endingVelocity = 0.0f) const
        {
            // Check if the targetPose is behind the currentPose
            const auto currentPose = odom.getPose();

            // Calculate starting velocity based on current velocity
            const auto currentVelocity = odom.getVelocity();
            const auto startingVelocity = currentVelocity.magnitude();
            Logger::info("Starting Velocity : " + std::to_string(startingVelocity) + " in/s");

            // Generate Path
            const auto distance = currentPose.distanceTo(targetPose);
            auto path = LinearPath::makeLinearPath(
                currentPose,
                targetPose);

            std::string filestring = "";
            for (int i=0; i < path.poses.size(); i++){
                filestring += path.poses[i].toString();
                filestring +="\n";
            }
            SDCardHandler sdHandler;
            sdHandler.writeFile("spline.csv", filestring);

            // Generate Trajectory
            const auto generator = TrajectoryGenerator({
                .startingVelocity = startingVelocity,
                .endingVelocity = endingVelocity
            });
            const auto trajectory = generator.calc(path);
            return trajectory;
        }

        /**
         * Tries to transform a pose using the assigned transformer, if any
         * @param pose - The pose to transform
         * @returns The transformed pose
         */
        Pose tryTransformPose(Pose pose) const
        {
            // If a transformer is assigned, transform the pose
            if (transformer)
                return transformer->transform(pose);

            // Otherwise, return the original pose
            return pose;
        }

        /// @brief The active transformer used to transform poses
        std::unique_ptr<PoseTransformer> transformer = nullptr;

        ChassisBase& chassis;
        OdomSource& odom;
    };
}
