#pragma once

#include "devils/devils.h"

namespace devils
{
    class ArmSystem
    {
    public:
        ArmSystem(SmartMotorGroup& armMotorA)
            : armMotorB(armMotorA)
        {
        }

        /**
         * Runs the arm in voltage mode.
         * @param voltage The voltage to run the arm at, from -1 to 1
         */
        void move(const float voltage)
        {
            armMotorB.move(voltage);
        }

    private:
        SmartMotorGroup& armMotorB;
    };
}