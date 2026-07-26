#pragma once

#include "devils/devilLib/devils.h"
#include <algorithm>

namespace devils
{
    PIDController liftPID(0.0005, 0.0001, 0.0001);

    /**
     * Represents the lift system of the robot.
     */
    class LiftSystem
    {
    public:
        LiftSystem(
            SmartMotorGroup& liftMotors, float maxHeight)
            : liftMotors(liftMotors), maxHeight(maxHeight)
        {
            liftMotors.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        }

    /**
     * Moves the lift at the specified speed.
     * @param speed The speed at which to move the lift.
     */
    void moveLift(const float speed)
    {
        liftMotors.move(speed);
    }

    /**
     * Moves the lift to a target position using a PID controller.
     * @param targetPosition The target position for the lift to move to (in pins).
     */
    void moveToPosition(const float targetPosition)
    {
        const float targetInches = convertToInches(targetPosition);
        const auto currentPosition = getPosition();
    
        std::clamp(currentPosition, 0.0f, maxHeight); // Clamp the current position to the range of the lift

        const float error = targetInches - currentPosition; // Calculate the error
        float output = liftPID.update(error); // Calculate the PID output

        moveLift(output); // Move the lift based on the PID output
    }

    /**
     * Stops the lift and chain bar motors.
     */
    void stopLift()
    {
        liftMotors.stop();
    }

    /**
     * Converts the position from pins to inches.
     * @param pins The position in pins.
     * @return The position in inches.
     */
    float convertToInches(const float pins)
    {
        return pins * 6.5f;
    }

    float getPosition()
    {
        float currentPosition = liftMotors.getPosition();
        float currentPositionInches = (currentPosition * 2 * M_PI / 50) * 0.3f; // Convert to inches
        return currentPositionInches;
    }

    void calibrateLift()
    {
        liftMotors.setPosition(0);
    }

    private:
        SmartMotorGroup& liftMotors;
        float maxHeight;
    };
}
