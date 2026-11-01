#pragma once

#include "devils/devilLib/devils.h"
#include "devils/devilLib/hardware/smartMotor.hpp"
#include <algorithm>
#include <cmath>

namespace devils
{
    PIDController PIDLeft(1.25, 0.000, 10);  // PID controller for the lift system
    PIDController PIDRight(1.25, 0.000, 10); // PID controller for the lift system
    class LiftSystem
    {
        // Member Variables
    private:
        // Hardware & Controllers
        SmartMotorGroup &leftMotors;
        SmartMotorGroup &rightMotors;

        // State Variables
        float maxHeight;
        float error = 0.0f;
        float targetInches = 0.0f;
        float radius;
        float circumference;

        // Constants
        const float ticks = 50.0f;
        const float gear_ratio = 6.0f;

        // Constructor
    public:
        LiftSystem(SmartMotorGroup &leftMotors, SmartMotorGroup &rightMotors, float maxHeight, float radius)
            : leftMotors(leftMotors), rightMotors(rightMotors), maxHeight(maxHeight), radius(radius)

        {
            circumference = 2.0f * (float)M_PI * radius;
            leftMotors.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // Set the brake mode to hold for both motors
            rightMotors.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        }

        // Member Functions

        // Setters and getters
        void setMaxHeight(float height)
        {
            maxHeight = height;
        }

        float getMaxHeight()
        {
            return maxHeight;
        }
        float getError()
        {
            return error;
        }

        /**
         * Moves the lift to a target position using a PID controller.
         * @param targetPosition The target position for the lift to move to (in inches).
         */
        void moveToPosition(const float targetPosition)
        {
            targetInches = targetPosition;
        }

        void update()
        {
            // currentPosition = getAveragePosition();

            float errorLeft = 0.0f;
            float errorRight = 0.0f;
            errorLeft = getTargetPosition() - getPosition(true);   // Calculate the error
            errorRight = getTargetPosition() - getPosition(false); // Calculate the error

            error = (errorLeft + errorRight) / 2.0f;
            // Han update the average lift error so getError() reports the current value.

            float outputLeft = PIDLeft.update(errorLeft);    // Calculate the PID output
            float outputRight = PIDRight.update(errorRight); // Calculate the PID output

            if (outputLeft < 0 && outputRight < 0)
            {
                float clampedOutputLeft = std::clamp(outputLeft, -1.0f, 1.0f);
                float clampedOutputRight = std::clamp(outputRight, -1.0f, 1.0f);

                clampedOutputLeft /= 2;
                clampedOutputRight /= 2;

                leftMotors.move(clampedOutputLeft);   // Move the lift based on the PID output
                rightMotors.move(clampedOutputRight); // Move the lift based on the PID output
            }
            else
            {
                leftMotors.move(outputLeft);   // Move the lift based on the PID output
                rightMotors.move(outputRight); // Move the lift based on the PID output
            }

        }

        float getTargetPosition()
        {
            targetInches = std::clamp(targetInches, 0.0f, maxHeight); // Clamp the current position to the range of the lift
            return targetInches;
        }

        /**
         * @param isLeft: true = left motor, false = right motor
         */
        float getPosition(bool isLeft)
        {
            float currentPosition = isLeft ? leftMotors.getPosition() : rightMotors.getPosition();
            float currentPositionInches = ((currentPosition * circumference) / (ticks * gear_ratio)); // Conversion from motor ticks to inches
            return currentPositionInches;
        }

        float getAveragePosition()
        {
            return (getPosition(true) + getPosition(false)) / 2.0f;
        }

        void calibrateLift()
        {
            leftMotors.setPosition(0);
            rightMotors.setPosition(0);
        }

        void stopLift()
        {
            leftMotors.stop();
            rightMotors.stop();
        }

        /**
         *@param pins: pass in 0.5 for half pin
         *@return The position in pins.
         */
        float pinsToInches(const float pins)
        {
            return pins * 6.75f;
        }

        float inchesToPins(const float inches)
        {
            return inches / 6.75f;
        }
    };
}
