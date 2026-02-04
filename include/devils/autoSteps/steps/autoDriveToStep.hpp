#pragma once

#include "../autoStep.hpp"
#include "../../geometry/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../controller/pidController.hpp"

namespace devils
{
    /**
     * Drives the robot linearly to a specific pose.
     * Rotates the robot to face the target pose, disregarding target rotation.
     */
    class AutoDriveToStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The PID parameters for translation. Uses delta inches as the error.
            PIDController::Options translationPID = {0.1, 0.0, 0.0};

            /// @brief The PID parameters for rotation. Uses delta radians as the error.
            PIDController::Options rotationPID = {0.05, 0.0, 0.0};

            /// @brief THe minimum speed in %
            float minSpeed = 0.0;

            /// @brief The maximum speed in %
            float maxSpeed = 0.5;

            /// @brief The maximum final distance to the target in inches
            float goalDist = 6.0;

            /// @brief The maximum final speed of the robot in inches/s. (Defaults to no limit)
            float goalSpeed = std::numeric_limits<float>::max();

            /// @brief The minimum distance from the target to apply rotation. If we are closer than this, we will not rotate to avoid oscillation.
            float minDistanceToRotate = 1.0;

            /// @brief The default options for the drive step.
            static Options defaultOptions;
        };

        /**
         * Drives the robot to a given pose.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param targetPose The target pose to drive to.
         * @param options The options for the drive step.
         */
        AutoDriveToStep(
            ChassisBase& chassis,
            OdomSource& odomSource,
            const Pose& targetPose,
            const Options& options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              rotationPID(options.rotationPID),
              translationPID(options.translationPID),
              targetPose(targetPose),
              options(options)
        {
        }

    protected:
        void onStart() override
        {
            // Reset PID Controllers
            rotationPID.reset();
        }

        void onUpdate() override
        {
            // Get Current State
            const Pose currentPose = odomSource.getPose();

            // Calculate distance to start and target
            float distanceToTarget = currentPose.distanceTo(targetPose);

            // Calculate target angle
            float targetAngleRads = std::atan2(
                targetPose.y - currentPose.y,
                targetPose.x - currentPose.x);

            // Calculate Dot Product
            const float currentDotTarget =
                cosf(currentPose.rotation) *
                (targetPose.x - currentPose.x) +
                sinf(currentPose.rotation) *
                (targetPose.y - currentPose.y);

            // Drive in reverse if the goal is behind us
            if (currentDotTarget < 0)
            {
                distanceToTarget = -distanceToTarget;
                targetAngleRads += M_PIF;
            }

            // Calculate Forward Speed
            float speed = getSpeed(distanceToTarget);

            // Calculate Turn Speed
            float turnSpeed = 0;
            if (std::fabs(distanceToTarget) > options.minDistanceToRotate)
            {
                // Difference in angle
                const float angleDiff = Math::angleDiff(targetAngleRads, currentPose.rotation);

                turnSpeed = rotationPID.update(angleDiff);
                turnSpeed = std::clamp(turnSpeed, -options.maxSpeed, options.maxSpeed);
            }

            // Move Chassis
            chassis.move(speed, turnSpeed, 0.0f);
        }

        void onStop() override
        {
            // Stop Chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            // Get Current State
            const Pose currentPose = odomSource.getPose();

            // Calculate distance to target pose
            const float distanceToTarget = currentPose.distanceTo(targetPose);

            // Check if we reached the goal
            return fabsf(distanceToTarget) < options.goalDist;
        }

        /**
         * Gets the speed at a given distance to the target.
         * @param distanceToTarget The distance to the target in inches
         * @returns The target speed in inches per second
         */
        virtual float getSpeed(float distanceToTarget)
        {
            // Calculate output speed
            float outputSpeed = translationPID.update(distanceToTarget);

            // Apply max speed
            outputSpeed = std::clamp(outputSpeed, -options.maxSpeed, options.maxSpeed);

            // Apply min speed
            if (std::fabs(outputSpeed) < options.minSpeed)
                outputSpeed = std::copysign(options.minSpeed, outputSpeed);

            return outputSpeed;
        }

        // Params
        ChassisBase& chassis;
        OdomSource& odomSource;
        PIDController rotationPID;
        PIDController translationPID;
        Pose targetPose = Pose();
        Options options;
    };

    // Define the default options
    AutoDriveToStep::Options AutoDriveToStep::Options::defaultOptions = AutoDriveToStep::Options();
}
