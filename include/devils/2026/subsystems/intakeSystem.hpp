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
        IntakeSystem(ADIPneumatic& intakeArms, SmartMotorGroup& intakeMotors)
            : intakeArms(intakeArms), intakeMotors(intakeMotors)
        {

        }

        void runIntake(float speed)
        {
            intakeMotors.move(speed);
        }

        void setArmsExtended(bool extended)
        {
            intakeArms.setExtended(extended);
        }

    private:
        ADIPneumatic intakeArms;
        SmartMotorGroup intakeMotors;
    };
    
}
