#pragma once

#include "devils/devilLib/devils.h"
#include <algorithm>

namespace devils
{
    PIDController liftPID(0.5, 0.0000, 0.0000); // PID controller for the lift system
    float targetLiftPosition; // Target position for the lift in inches
    float error;
    float currentPosition;
    float targetInches;

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
        targetInches = convertToInches(targetPosition);
        targetInches = std::clamp(targetInches, 0.0f, maxHeight); // Clamp the current position to the range of the lift

        targetLiftPosition = targetInches; // Store the target position for telemetry

    }

    void update()
    {
        currentPosition = getPosition();
        error = targetInches - getPosition(); // Calculate the error
        float output = liftPID.update(error); // Calculate the PID output

        moveLift(output); // Move the lift based on the PID output
    }
    
    float getTargetPosition() const
    {
        return targetLiftPosition;
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

    float convertToPins(const float inches)
    {
        return inches / 6.5f;
    }

    float getPosition()
    {
        float currentPosition = liftMotors.getPosition();
        float currentPositionInches = ((currentPosition * 2 * M_PI * 0.3f) / (50 * 6.0f)); // Convert to inches
        return currentPositionInches;
    }

    float getError(){
        return error;
    }

    void calibrateLift()
    {
        liftMotors.setPosition(0);
    }

    void setMaxHeight(float height)
    {
        maxHeight = height;
    }

    private:
        SmartMotorGroup& liftMotors;
        float maxHeight;
    };
}
