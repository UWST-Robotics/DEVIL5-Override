#pragma once

#include "../subsystems/intakeSystem.hpp"

namespace devils
{
    class AutoIntakeStep : public AutoStep
    {
    public:
        AutoIntakeStep(
            IntakeSystem& intake,
            const float targetSpeed = 1.0)
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
        void setTargetSpeed(const float speed)
        {
            targetSpeed = speed;
        }

    private:
        IntakeSystem& intake;
        float targetSpeed = 1.0;
    };
}
