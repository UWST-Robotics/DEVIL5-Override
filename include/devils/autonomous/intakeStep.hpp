#pragma once

#include "../subsystems/intakeSystem.hpp"

namespace devils
{
    class IntakeStep : public AutoStep
    {
    public:
        IntakeStep(
            IntakeSystem& intake,
            const float targetSpeed = 1.0)
            : intake(intake),
              targetSpeed(targetSpeed)
        {
        }

        void onUpdate() override
        {
            if (matchloading)
                intake.matchLoad(targetSpeed);
            else
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

        /**
         * Toggles the match loading state of the intake.
         * @return The new match loading state.
         */
        bool setMatchloading()
        {
            matchloading = !matchloading;
            return matchloading;
        }

    private:
        IntakeSystem& intake;
        float targetSpeed = 1.0;
        bool matchloading = false;
    };
}