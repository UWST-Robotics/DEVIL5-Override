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
    class RotateStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The PID parameters to snap to an angle. Used to correct drift from the motion profile.
            PIDController::Options pidParams = {1, 0.0, 0.0};
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
        RotateStep(
            ChassisBase& chassis,
            OdomSource& odomSource,
            const float targetAngle,
            const Options& options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              targetAngle(targetAngle),
              pidController(options.pidParams),
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
            pidController.reset();
        }

        void onUpdate() override
        {
            // Get PID output
            const auto currentAngle = odomSource.getPose().rotation;
            const auto currentAngleRelative = Units::diffRad(currentAngle, targetAngle);
            const auto pidOutput = pidController.update(currentAngleRelative);

            Logger::info("Current Angle: " + std::to_string(currentAngle));
            Logger::warn("Target Angle: " + std::to_string(targetAngle));
            Logger::error("Angle Difference: " + std::to_string(Units::diffRad(currentAngle, targetAngle)));

            // Combine the outputs and apply to the chassis
            chassis.move(0.0f, pidOutput, 0.0f);
        }

        void onStop() override
        {
            chassis.stop();
        }

        bool checkFinished() override
        {
            //return (getGoalDistance() < 0.05f); // Check if the remaining angle to the target is less than or equal to 0.01 radians (about 0.57 degrees)
            return false;
        }
        
        ChassisBase& chassis;
        OdomSource& odomSource;
        float targetAngle = 0;
        PIDController pidController;
        Options options;

    private:
        /**
         * Gets the remaining angle to the target angle based on the current angle from odometry and the target angle.
         * @return the remaining angle to the target angle in radians.
         */
        float getGoalDistance() const
        {
            const auto currentAngle = odomSource.getPose().rotation;
            return Units::diffRad(targetAngle, currentAngle);
        }
        
    };

    // Initialize Default Options
    RotateStep::Options RotateStep::Options::defaultOptions = Options();
}
