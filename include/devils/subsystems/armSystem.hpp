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
            liftMotors.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            chainBarMotors.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
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

    /**
     * Stops the lift and chain bar motors.
     */
    void stopLift()
    {
        liftMotors.stop();
        chainBarMotors.stop();
    }

    private:
        static constexpr float CHAINBAR_LIFT_SPEED_RATIO = 1.0f /*0.375f*/; // The chain bar moves at 0.375 times the speed of the lift

        SmartMotorGroup& liftMotors;
        SmartMotorGroup& chainBarMotors;
    };
}
