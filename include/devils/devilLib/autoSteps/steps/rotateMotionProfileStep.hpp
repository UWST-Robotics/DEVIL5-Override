#pragma once

#include "../autoStep.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../geometry/math.hpp"
#include "../../controller/pidController.hpp"
#include "../../controller/profiledPIDController.hpp"

namespace devils
{
    /**
     * Rotates the robot to a specific angle along its center of rotation.
     */
    class RotateMotionProfileStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The PID parameters to snap to an angle. Used to correct drift from the motion profile.
            PIDController::Options pidParams = {0.8, 0.0, 0.0};
            
            /// @brief The constraints for the motion profile.
            TrapezoidMotionProfile::Constraints motionProfileConstraints = {16.0f, 8.0f, 16.0f};
            
            /// @brief The feedforward option to apply based on the motion profile.
            MotorFeedforward::Options feedforwardOptions = {0.0f, 0.09f, 0.005f};
            
            /// @brief The amount of time to add to the remaining time of the motion profile to account for feedback latency
            float feedbackLatency = 0.07f; // 70 ms
            
            /// @brief The starting rotational velocity for the motion profile
            float startingVelocity = 0.0f;
            
            /// @brief The ending rotational velocity for the motion profile
            float endingVelocity = 0.0f;

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
        RotateMotionProfileStep(
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
              feedforwardController(options.feedforwardOptions),
              options(options)
        {
            // Set the goal distance for the motion profile based on the target angle and the current angle from odometry.
            pidController.setGoal(
                getGoalDistance(),
                options.startingVelocity,
                options.endingVelocity);
            
            // Set the feedback delay for the PID controller to account for any latency in the odometry or the control loop.
            pidController.setFeedbackDelay(options.feedbackLatency);
        }

        Options& getOptions()
        {
            return options;
        }

    protected:
        void onStart() override
        {
            pidController.reset();
            startingRotation = odomSource.getPose().rotation;
        }

        void onUpdate() override
        {
            // Get profiled PID output
            const auto currentAngle = odomSource.getPose().rotation;
            const auto currentAngleRelative = Units::diffRad(currentAngle, startingRotation);
            const auto pidOutput = pidController.update(currentAngleRelative);
            
            // Get feedforward output
            const auto setpoint = pidController.getSetpoint();
            const auto feedforwardOutput = feedforwardController.update(setpoint.velocity, setpoint.acceleration);
            
            // Combine the outputs and apply to the chassis
            chassis.move(0.0f, feedforwardOutput + pidOutput, 0.0f);
        }

        void onStop() override
        {
            chassis.stop();
        }

        bool checkFinished() override
        {
            return pidController.getTimeRemaining() <= 0;
        }
        
        ChassisBase& chassis;
        OdomSource& odomSource;
        float targetAngle = 0;
        float startingRotation = 0;
        ProfiledPIDController pidController;
        MotorFeedforward feedforwardController;
        Options options;

    private:
        /**
         * Gets the remaining angle to the target angle based on the current angle from odometry and the target angle.
         * @return the remaining angle to the target angle in radians.
         */
        float getGoalDistance() const
        {
            const auto currentAngle = odomSource.getPose().rotation;
            return options.useMinimumDistance ? 
                Units::diffRad(targetAngle, currentAngle) : 
                targetAngle - currentAngle;
        }
        
    };

    // Initialize Default Options
    RotateMotionProfileStep::Options RotateMotionProfileStep::Options::defaultOptions = Options();
}
