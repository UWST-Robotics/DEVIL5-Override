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
        IntakeSystem(ADIPneumatic& intakeArmLeft, ADIPneumatic& intakeArmRight, SmartMotorGroup& intakeMotors)
            : intakeArmLeft(intakeArmLeft), intakeArmRight(intakeArmRight), intakeMotors(intakeMotors)
        {

        }

        void runIntake(float speed)
        {
            intakeMotors.move(speed);
        }

        void setArmsExtended(bool extended)
        {
            intakeArmLeft.setExtended(extended);
            intakeArmRight.setExtended(extended);
        }

    private:
        ADIPneumatic intakeArmLeft;
        ADIPneumatic intakeArmRight;
        SmartMotorGroup intakeMotors;
    };
    
}
