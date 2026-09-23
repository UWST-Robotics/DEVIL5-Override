#pragma once

#include "devils/devilLib/devils.h"

namespace devils
{
    /**
     * Represents the intake arm and claw system of the robot.
     */
    class GrabberSystem
    {
    public:
        GrabberSystem(
            SmartMotorGroup& rollers)
            : rollers(rollers)
        {
        }

        void runGrabber(float speed) {
            rollers.move(speed);
        }

        SmartMotorGroup& rollers;
    };
}