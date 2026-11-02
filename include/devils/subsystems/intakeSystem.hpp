#pragma once

#include "devils/devilLib/devils.h"
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
            SmartMotorGroup& mainRollers,
            SmartMotorGroup& throughtakeRollers)
            : mainRollers(mainRollers),
              throughtakeRollers(throughtakeRollers)
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
            mainRollers.move(speed);
            if (speed < 0)
                throughtakeRollers.move(speed);
        }

        private:
        SmartMotorGroup& mainRollers;
        SmartMotorGroup& throughtakeRollers;
    };
}
