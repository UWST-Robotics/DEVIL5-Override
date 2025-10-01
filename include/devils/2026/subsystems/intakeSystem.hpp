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
        /**
         * Creates a new instance of the intake system.
         * @param intakeMotors The motors controlling the intake.
         */
        IntakeSystem(SmartMotorGroup &intakeMotors)
            : intakeMotors(intakeMotors)
        {
            intakeMotors.setPosition(0);
        }

        /**
         * Stops all motors in the intake.
         */
        void stop()
        {
            for (auto motor : intakeMotors.getMotors())
                motor->moveVoltage(0);
        }

        /**
         * Moves intake using speed
         * @param speed - Voltage to run intake between -1 and 1.
         */
        void move(float speed)
        {
            speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);
            intakeMotors.moveVoltage(speed);
        }

    private:
        static constexpr float MAX_SPEED = 1.0;  // %
        static constexpr float MIN_SPEED = -0.6; // %

        // Hardware
        SmartMotorGroup &intakeMotors;
    };
}