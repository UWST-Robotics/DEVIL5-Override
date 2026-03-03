#pragma once

#include "devils/devils.h"
#include <algorithm>

namespace devils
{
    /**
     * Represents the intake arm and claw system of the robot.
     */
    class IntakeSystem
    {
    public:
        IntakeSystem(
            ADIPneumaticGroup& intakeArms,
            SmartMotorGroup& sideRollers,
            SmartMotorGroup& topRollers)
            : intakeArms(intakeArms),
              sideRollers(sideRollers),
              topRollers(topRollers)
        {
        }

        /**
         * Runs the intake rollers at a specified speed.
         * The speed should be between -1 and 1, where positive values run the rollers inwards to intake objects and negative values run the rollers outwards to expel objects.
         * The speed is limited to prevent the rollers from running too fast.
         * @param speed - The speed to run the intake rollers at, from -1 to 1. 
         */
        void runIntake(float speed)
        {
            if (isStickStalled)
            {
                topRollers.move(-IDLE_ROLLER_SPEED);
                sideRollers.move(0);
                return;;
            }

            if (std::abs(speed) < INPUT_DEADZONE)
            {
                sideRollers.move(0);
                topRollers.move(IDLE_ROLLER_SPEED);
                return;
            }

            // Limit the speed to prevent the rollers from running too fast
            speed = std::clamp(speed, -1.0f, 1.0f);

            sideRollers.move(speed * SIDE_ROLLER_SPEED);
            topRollers.move(speed * TOP_ROLLER_SPEED);
        }

        /**
         * Sets whether the intake arms are extended or not.
         * @param extended - True to extend the intake arms, false to retract them.
         */
        void setArmsExtended(const bool extended) const
        {
            intakeArms.setExtended(extended);
        }

        /**
         * Sets the stick stalled state, which can be used to prevent the intake from running when the stick is stalled during retraction.
         * @param isStalled - True if the stick is stalled, false otherwise.
         */
        void setStickStalled(const bool isStalled)
        {
            this->isStickStalled = isStalled;
        }

    private:
        static constexpr float TOP_ROLLER_SPEED = 0.75f;
        static constexpr float SIDE_ROLLER_SPEED = 0.75f;

        static constexpr float IDLE_ROLLER_SPEED = 0.5f;
        static constexpr float INPUT_DEADZONE = 0.1f;

        bool isStickStalled = false;

        ADIPneumaticGroup& intakeArms;
        SmartMotorGroup& sideRollers;
        SmartMotorGroup& topRollers;
    };
}
