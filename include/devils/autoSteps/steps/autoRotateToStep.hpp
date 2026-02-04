#pragma once

#include "../autoStep.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../geometry/math.hpp"
#include "../../controller/pidController.hpp"

namespace devils
{
    /**
     * Rotates the robot to a specific angle along its center of rotation.
     */
    class AutoRotateToStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The PID parameters to snap to an angle. Uses delta radians as the error.
            PIDController::Options pidParams = {0.1, 0.0, 0.0};

            /// @brief The minimum speed in %
            float minSpeed = 0.1;

            /// @brief The maximum speed in %
            float maxSpeed = 0.6;

            /// @brief The distance to the goal in radians
            float goalDist = 0.015;

            /// @brief The maximum goal speed of the robot in rad/s. (Defaults to no limit)
            float goalSpeed = std::numeric_limits<float>::max();

            /// @brief Setting this to false will rotate to the absolute angle instead of the minimum distance.
            bool useMinimumDistance = true;

            /// @brief The default options for the rotational step.
            static Options defaultOptions;
        };

        /**
         * Rotates the robot to a specific angle along its center of rotation.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param targetAngle The angle to rotate to in radians.
         * @param options The options for the rotational step.
         */
        AutoRotateToStep(
            ChassisBase& chassis,
            OdomSource& odomSource,
            const float targetAngle,
            const Options& options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              rotationPID(options.pidParams),
              targetAngle(targetAngle),
              options(options)
        {
        }

        Options& getOptions()
        {
            return options;
        }

    protected:
        void onStart() override
        {
            // Reset Finished
            this->isAtGoal = false;

            // Reset PID
            rotationPID.reset();
        }

        void onUpdate() override
        {
            // Get Current Pose
            const Pose currentPose = odomSource.getPose();
            const float currentVelocity = odomSource.getVelocity().rotation;

            // Calculate distance to start and target
            const float currentAngle = currentPose.rotation;
            const float distanceToTarget = angleDiff(targetAngle, currentAngle);

            // Check if we are at the goal
            const bool isAtGoalPose = fabsf(distanceToTarget) < options.goalDist;
            const bool isAtGoalVelocity = fabsf(currentVelocity) < options.goalSpeed;
            isAtGoal = isAtGoalPose && isAtGoalVelocity;

            // Calculate Speed
            float speed = rotationPID.update(distanceToTarget);
            speed = std::clamp(speed, -options.maxSpeed, options.maxSpeed); // Clamp to max speed
            speed = std::copysign(std::max(fabsf(speed), options.minSpeed), speed); // Clamp to min speed

            // Move Chassis
            chassis.move(0.0f, speed, 0.0f);
        }

        void onStop() override
        {
            // Stop Chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            return isAtGoal;
        }

    protected:
        // State
        bool isAtGoal = false;

        // Params
        ChassisBase& chassis;
        OdomSource& odomSource;
        PIDController rotationPID;
        float targetAngle = 0;
        Options options;

    private:
        static constexpr float POST_DRIVE_DELAY = 50; // ms

        /**
         * Gets the angle difference between two angles.
         * Uses the minimum distance if the option is enabled.
         * Otherwise, returns the difference.
         * @param a The first angle.
         * @param b The second angle.
         * @return The difference between the two angles.
         */
        float angleDiff(const float a, const float b) const
        {
            if (options.useMinimumDistance)
                return Math::angleDiff(a, b);
            return a - b;
        }
    };

    // Initialize Default Options
    AutoRotateToStep::Options AutoRotateToStep::Options::defaultOptions = AutoRotateToStep::Options();
}
