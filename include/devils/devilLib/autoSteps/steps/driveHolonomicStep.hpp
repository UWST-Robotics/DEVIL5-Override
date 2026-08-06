#pragma once

#include "../autoStep.hpp"
#include "../../geometry/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../controller/pidController.hpp"
#include "../../trajectory/trajectory.hpp"
//#include "../../utils/sdCardHandler.hpp"

namespace devils
{
    /**
     * Drives a holonomic robot linearly to a specific pose.
     */
    class DriveHolonomicStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The PID parameters for translation in the X (forward/backward) direction. Uses delta inches as the error.
            PIDController::Options translationPIDX = {0.15, 0.0, 0.0};

            /// @brief The PID parameters for translation in the Y (left/right) direction. Uses delta inches as the error.
            PIDController::Options translationPIDY = {0.15, 0.0, 0.0};

            /// @brief The PID parameters for rotation. Uses delta radians as the error.
            PIDController::Options rotationPID = {1.0, 0.0, 0.0};

            /// @brief THe minimum speed in %
            float minSpeed = 0.0;

            /// @brief The maximum speed in %
            float maxSpeed = 0.5;

            /// @brief The maximum final distance to the target in inches
            float goalDist = 6.0;

            /// @brief The maximum final speed of the robot in inches/s. (Defaults to no limit)
            float goalSpeed = std::numeric_limits<float>::max();

            /// @brief Time in seconds of odometry latency. This is the time it takes for the odometry to update after the robot moves.
            float sensorLatency = 0.02;

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
        DriveHolonomicStep(
            ChassisBase& chassis,
            OdomSource& odomSource,
            const std::shared_ptr<Trajectory>& trajectory,
            const Options& options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              trajectory(trajectory),
              internalTimer(trajectory->duration()),
              rotationPID(options.rotationPID),
              translationPIDX(options.translationPIDX),
              translationPIDY(options.translationPIDY),
              options(options)
        {
        }

    protected:

        //SDCardHandler cardHandler;

        void onStart() override
        {
            // Reset PID Controllers
            rotationPID.reset();
            translationPIDX.reset();
            translationPIDY.reset();

            internalTimer.start();
        }

        void onUpdate() override
        {
            // Get the current time
            const auto t = internalTimer.getElapsedTime();

            // Get current setpoint
            const auto setpoint = trajectory->getStateAt(t);
            const auto feedbackSetpoint = trajectory->getStateAt(t - options.sensorLatency);

            // Get current position
            const auto currentPosition = odomSource.getPose();

            // Calculate error to setpoint
            const auto error = feedbackSetpoint.pose - currentPosition;

            // Calculate local error
            // This is the error relative to the robot's current rotation
            // such that localError.x is aligned with the front of the robot and
            // localError.y is aligned with the side of the robot.
            const auto localError = Pose(
                error.x * cosf(currentPosition.rotation) + error.y * sinf(currentPosition.rotation),
                (-error.x * sinf(currentPosition.rotation) + error.y * cosf(currentPosition.rotation)),
                Units::diffRad(feedbackSetpoint.pose.rotation, currentPosition.rotation));

            // Move Chassis using PID controllers with local error as the input
            chassis.move(translationPIDX.update(localError.x), rotationPID.update(localError.rotation), translationPIDY.update(localError.y));
        }

        void onStop() override
        {
            // Stop Chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            /*
            // Get Current State
            const Pose currentPose = odomSource.getPose();

            // Calculate distance to target pose
            const float distanceToTarget = currentPose.distanceTo(targetPose);

            // Check if we reached the goal
            return fabsf(distanceToTarget) < options.goalDist;
            */
            // TODO: Make this criteria smarter
            return internalTimer.getIsFinished();
        }

        /**
         * Gets the speed at a given distance to the target.
         * @param distanceToTarget The distance to the target in inches
         * @returns The target speed in inches per second
         */
        /*
        virtual float getSpeed(const float distanceToTarget)
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
        */
        // Params
        ChassisBase& chassis;
        OdomSource& odomSource;
        PIDController rotationPID;
        PIDController translationPIDX;
        PIDController translationPIDY;
        std::shared_ptr<Trajectory> trajectory;
        Timer internalTimer;
        Options options;
    };

    // Define the default options
    DriveHolonomicStep::Options DriveHolonomicStep::Options::defaultOptions = DriveHolonomicStep::Options();
}
