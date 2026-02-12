#pragma once

#include "../autoStep.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../geometry/math.hpp"
#include "../../controller/pidController.hpp"
#include "devils/controller/profiledPIDController.hpp"

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
            /// @brief The PID parameters to snap to an angle. Used as a feedback controller.
            PIDController::Options pidParams = {0.5, 0.0, 0.0};
            
            /// @brief The constraints for the motion profile. Used as a feedforward controller
            TrapezoidMotionProfile::Constraints motionProfileConstraints = {3.0f, 3.0f, 3.0f};

            /**
             * If true, the robot will try to rotate to the angle using the fastest path possible.
             * For example if the robot is at 10 degrees and the target angle is 350 degrees, the robot will rotate -20 degrees instead of +340 degrees.
             * 
             * If false, the robot will try to match the angle exactly, even if it means taking a longer path.
             * For example if the robot is at 10 degrees and the target angle is 350 degrees, the robot will rotate +340 degrees instead of -20 degrees.
             */
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
              targetAngle(targetAngle),
              pidController(options.pidParams,
                            options.motionProfileConstraints,
                            0),
              options(options)
        {
            pidController.setGoal(getGoalAngle());
        }

        Options& getOptions()
        {
            return options;
        }

    protected:
        void onStart() override
        {
            pidController.reset();
        }

        void onUpdate() override
        {
            // Get profiled PID output
            const auto currentAngle = odomSource.getPose().rotation;
            const auto velocity = pidController.getState(currentAngle);
            chassis.move(0.0f, velocity, 0.0f);
        }

        void onStop() override
        {
            // Stop Chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            return pidController.getTimeRemaining() <= 0;
        }
        
        ChassisBase& chassis;
        OdomSource& odomSource;
        float targetAngle = 0;
        ProfiledPIDController pidController;
        Options options;

    private:
        static constexpr float POST_DRIVE_DELAY = 50; // ms

        /**
         * Gets the goal angle to rotate to based on the current angle and the target angle, taking into account the `useMinimumDistance` option.
         * @return The goal angle to rotate to in radians.
         */
        float getGoalAngle() const
        {
            const auto currentAngle = odomSource.getPose().rotation;
            return options.useMinimumDistance ? currentAngle + Math::angleDiff(targetAngle, currentAngle) : targetAngle;
        }
    };

    // Initialize Default Options
    AutoRotateToStep::Options AutoRotateToStep::Options::defaultOptions = Options();
}
