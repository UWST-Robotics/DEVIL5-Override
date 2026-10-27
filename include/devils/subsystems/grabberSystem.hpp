#pragma once

#include "devils/devilLib/devils.h"

namespace devils
{
    /**
     * Represents the claw system of the robot.
     */
    class GrabberSystem
    {
    public:
        GrabberSystem(
            SmartMotorGroup& rollers)
            : rollers(rollers)
        {
        }

        /**
         * Moves the grabber at a target speed
         */
        void moveGrabber(float speed) {
            rollers.move(speed);
        }

        SmartMotorGroup& rollers;
    };
}