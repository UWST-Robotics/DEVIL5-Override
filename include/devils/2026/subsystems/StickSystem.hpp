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

        void moveFast()
        {
            moveToPosition(EXPECTED_POSITION_UP, FAST_SPEED);
        }
        void moveSlow()
        {
            moveToPosition(EXPECTED_POSITION_UP, SLOW_SPEED);
        }
        void retract()
        {
            moveToPosition(EXPECTED_POSITION_DOWN, RETRACTION_SPEED);
        }

        void moveToPosition(float targetPosition, float speed)
        {
            if (pto.getExtended()) return; // Don't move stick if PTO is extended

            const float currentPosition = stickMotors.getPosition(); // Get the current position of the stick
            const float error = targetPosition - currentPosition; // Calculate the error
            float output = stickPID.update(error); // Calculate the PID output
            output = Math::smallestMagnitude({output, speed}); // Limit the output to the maximum speed
            stickMotors.move(output); // Move the stick based on the PID output
        }

    private:
        ADIPneumatic pto;
        SmartMotorGroup stickMotors;
        PIDController stickPID{0.006f, 0.0f, 0.1f}; // PID controller for stick position control

        static constexpr float FAST_SPEED = 0.7f;
        static constexpr float SLOW_SPEED = 0.5f;
        static constexpr float RETRACTION_SPEED = -0.35f;
        static constexpr float EXPECTED_POSITION_DOWN = 0.0f; // Expected position of the stick when fully retracted (in ticks)
        static constexpr float EXPECTED_POSITION_UP = 1000.0f; // Expected position of the stick when fully extended (in ticks)
    };
    
}

