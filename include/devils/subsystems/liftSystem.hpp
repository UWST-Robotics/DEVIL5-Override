#pragma once

#include "devils/devilLib/devils.h"
#include "devils/devilLib/hardware/smartMotor.hpp"
#include <algorithm>

namespace devils
{
    PIDController liftPIDLeft(0.5, 0.0000, 0.0000); // PID controller for the lift system
    PIDController liftPIDRight(0.5, 0.0000, 0.0000); // PID controller for the lift system
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
            SmartMotor& leftMotor, SmartMotor& rightMotor, float maxHeight)
            : leftMotor(leftMotor), rightMotor(rightMotor), maxHeight(maxHeight)
        {
            leftMotor.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            rightMotor.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        }

    /**
     * Moves the lift to a target position using a PID controller.
     * @param targetPosition The target position for the lift to move to (in inches).
     */
    void moveToPosition(const float targetPosition)
    {
        targetInches = targetPosition;
        targetInches = std::clamp(targetInches, 0.0f, maxHeight); // Clamp the current position to the range of the lift
    }

    void update()
    {
        currentPosition = getAveragePosition();
        float errorLeft = targetInches - getPosition(true); // Calculate the error
        float errorRight = targetInches - getPosition(false); // Calculate the error
        error = (errorLeft + errorRight) / 2.0f;
        float outputLeft = liftPIDLeft.update(errorLeft); // Calculate the PID output
        float outputRight = liftPIDRight.update(errorRight); // Calculate the PID output

        leftMotor.move(outputLeft); // Move the lift based on the PID output
        rightMotor.move(outputRight); // Move the lift based on the PID output
    }
    
    float getTargetPosition() const
    {
        return targetInches;
    }

    /**
     * Stops the lift and chain bar motors.
     */
    void stopLift()
    {
        leftMotor.stop();
        rightMotor.stop();
    }

    /**
     * Converts the position from pins to inches.
     * @param pins The position in pins.
     * @return The position in inches.
     */
    float convertPinsToInches(const float pins)
    {
        return pins * 6.5f;
    }

    float convertHalfPinsToInches(const float pins)
    {
        return pins * 3.25f;
    }


    float convertToPins(const float inches)
    {
        return inches / 6.5f;
    }

    float convertToHalfPins(const float inches)
    {
        return inches / 3.25f;
    }

    float getPosition(bool isLeft)
    {
        float currentPosition = isLeft ? leftMotor.getPosition() : rightMotor.getPosition();
        float currentPositionInches = ((currentPosition * 2.0f * M_PIF * 0.3f) / (50.0f * 6.0f)); // Convert to inches
        return currentPositionInches;
    }

    float getAveragePosition()
    {
        return (getPosition(true) + getPosition(false)) / 2.0f;
    }

    float getError(){
        return error;
    }

    void calibrateLift()
    {
        leftMotor.setPosition(0);
    }

    void setMaxHeight(float height)
    {
        maxHeight = height;
    }

    float getMaxHeight()
    {
        return maxHeight;
    }

    private:
        SmartMotor& leftMotor;
        SmartMotor& rightMotor;
        float maxHeight;
    };
}
