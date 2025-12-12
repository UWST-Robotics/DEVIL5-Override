#pragma once
#include "../subsystems/IntakeSystem.hpp"

namespace devils
{
    class AutoIntakeStep : public AutoStep
    {
    public:
        AutoIntakeStep(IntakeSystem &intake, float targetSpeed = 1.0)
            : intake(intake),
              targetSpeed(targetSpeed)
        {
        }

        void onUpdate() override
        {
            intake.runIntake(targetSpeed);
        }

        /**
         * Sets the target speed of the intake.
         * @param speed The target speed of the intake.
         */
        void setTargetSpeed(double speed)
        {
            targetSpeed = speed;
        }

    private:
        IntakeSystem &intake;
        double targetSpeed = 1.0;
    };
}