#pragma once

#include "devils/devilLib/devils.h"
#include <algorithm>
#include <cmath>

namespace devils
{
<<<<<<< Updated upstream
    PIDController liftPID(0.1, 0.0000, 0.0000); // PID controller for the lift system
    float targetLiftPosition; // Target position for the lift in inches

    /**
     * Represents the lift system of the robot.
     */
=======
    PIDController PIDLeft(0.5, 0.0000, 0.0000);  // PID controller for the lift system
    PIDController PIDRight(0.5, 0.0000, 0.0000); // PID controller for the lift system
>>>>>>> Stashed changes
    class LiftSystem
    {
        // Member Variables
    private:
        // Hardware & Controllers
        SmartMotor &leftMotor;
        SmartMotor &rightMotor;

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
<<<<<<< Updated upstream
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
        targetLiftPosition = targetPosition; // Store the target position for telemetry
        
        const float targetInches = convertToInches(targetPosition);
        const auto currentPosition = getPosition();
    
        std::clamp(currentPosition, 0.0f, maxHeight); // Clamp the current position to the range of the lift

        const float error = targetInches - currentPosition; // Calculate the error
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
=======
        LiftSystem(SmartMotor &leftMotor, SmartMotor &rightMotor, float maxHeight, float radius)
            : leftMotor(leftMotor), rightMotor(rightMotor), maxHeight(maxHeight), radius(radius)

        {
            circumference = 2.0f * (float)M_PI * radius;
            leftMotor.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // Set the brake mode to hold for both motors
            rightMotor.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
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

            leftMotor.move(outputLeft);   // Move the lift based on the PID output
            rightMotor.move(outputRight); // Move the lift based on the PID output
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
            float currentPosition = isLeft ? leftMotor.getPosition() : rightMotor.getPosition();
            float currentPositionInches = ((currentPosition * circumference) / (ticks * gear_ratio)); // Conversion from motor ticks to inches
            return currentPositionInches;
        }

        float getAveragePosition()
        {
            return (getPosition(true) + getPosition(false)) / 2.0f;
        }

        void calibrateLift()
        {
            leftMotor.setPosition(0);
            rightMotor.setPosition(0);
        }

        void stopLift()
        {
            leftMotor.stop();
            rightMotor.stop();
        }

        /**
         *@param pins: pass in 0.5 for half pin
         *@return The position in pins.
         */
        double pinsToInches(const float pins)
        {
            return pins * 6.5f;
        }

        double inchesToPins(const float inches)
        {
            return inches / 6.5f;
        }
>>>>>>> Stashed changes
    };
}
