#pragma once

#include "../subsystems/intakeSystem.hpp"
#include "../subsystems/StickSystem.hpp"

namespace devils
{
    class AutoIntakeStep : public AutoStep
    {
    public:
        AutoIntakeStep(
            IntakeSystem& intake,
            StickSystem& stick,
            const float targetSpeed = 1.0)
            : intake(intake),
              stick(stick),
              targetSpeed(targetSpeed)
        {
        }

        void onUpdate() override
        {
            intake.runIntake(targetSpeed);
            intake.setStickStalled(stick.checkStalled());
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
        StickSystem& stick;
        float targetSpeed = 1.0;
    };
}
