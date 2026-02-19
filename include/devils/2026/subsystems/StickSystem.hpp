#pragma once

#include "devils/devils.h"
#include <algorithm>

namespace devils
{
    /**
     * Represents the intake arm and claw system of the robot.
     */
    class StickSystem
    {

    public:
        StickSystem(ADIPneumatic& pto, SmartMotorGroup& stickMotors)
            : pto(pto), stickMotors(stickMotors)
        {

        }

        void setPTOExtended(bool extended)
        {
            pto.setExtended(extended);
        }

        void moveStick(float speed)
        {
            if (pto.getExtended()) return; // Don't move stick if PTO is extended
            stickMotors.move(speed);
        }

        void moveFast()
        {
            moveStick(FAST_SPEED);
        }
        void moveSlow()
        {
            moveStick(SLOW_SPEED);
        }
        void retract()
        {
            moveStick(RETRACTION_SPEED);
        }

    private:
        ADIPneumatic pto;
        SmartMotorGroup stickMotors;

        static constexpr float FAST_SPEED = 0.7f;
        static constexpr float SLOW_SPEED = 0.5f;
        static constexpr float RETRACTION_SPEED = -0.35f;
    };
    
}

