#pragma once

#include "devils/devils.h"
#include <algorithm>

namespace devils
{
    /**
     * Represents the intake arm and claw system of the robot.
     */
    class ArmSystem
    {
    public:
        ArmSystem(
            SmartMotorGroup& liftMotors,
            SmartMotorGroup& chainBarMotors)
            : liftMotors(liftMotors),
              chainBarMotors(chainBarMotors)
        {
        }

    /**
     * Moves the lift and chain bar at the specified speed.
     * @param speed The speed at which to move the lift and chain bar.
     */
    void moveLift(const float speed)
    {
        liftMotors.move(speed);
        chainBarMotors.move(speed * CHAINBAR_LIFT_SPEED_RATIO);
    }

    private:
        static constexpr float CHAINBAR_LIFT_SPEED_RATIO = 2.0f; // The chain bar moves at twice the speed of the lift

        SmartMotorGroup& liftMotors;
        SmartMotorGroup& chainBarMotors;
    };
}
