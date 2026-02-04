#pragma once

#include "../hardware/smartMotorGroup.hpp"

namespace devils
{
    /**
     * Controls 2 sets of motors symmetrically using encoders.
     * Acts like a differential drive, but uses encoders to adjust the speed of each side.
     * The motor that is ahead will be slowed down proportionally to its distance ahead.
     */
    class SymmetricControl
    {
    public:
        SymmetricControl(SmartMotorGroup &leftMotors, SmartMotorGroup &rightMotors)
            : leftMotors(leftMotors),
			  rightMotors(rightMotors)
        {
        }

        /**
         * Resets the offsets of the left and right motors.
         */
        void resetOffsets()
        {
            leftOffset = leftMotors.getPosition();
            rightOffset = rightMotors.getPosition();
        }

        /**
         * Drives the motors in a symmetric manner.
         * @param speed The speed to drive the motors at, from -1 to 1.
         */
        void drive(float speed, float horizontalSpeed)
        {
            // Clamp the speed to the range [-1, 1]
            speed = std::clamp(speed, -1.0, 1.0);
            horizontalSpeed = std::clamp(horizontalSpeed, -1.0, 1.0);

            // Apply horizontal deadzone
            if (std::abs(horizontalSpeed) < HORIZONTAL_DEADZONE)
                horizontalSpeed = 0.0;

            // Adjust encoder offsets
            leftOffset -= horizontalSpeed * HORIZONTAL_ENCODER_SPEED;
            rightOffset += horizontalSpeed * HORIZONTAL_ENCODER_SPEED;

            // Fetch the encoder values and subtract the offsets
            float leftEncoder = leftMotors.getPosition() - leftOffset;
            float rightEncoder = rightMotors.getPosition() - rightOffset;

            // Check which motor is behind
            bool leftMotorBehind = leftEncoder < rightEncoder;
            bool reverse = speed < 0;
            if (reverse)
                leftMotorBehind = !leftMotorBehind;

            // Calculate the speed scaling factor
            float behindEncoder = leftMotorBehind ? leftEncoder : rightEncoder;
            float aheadEncoder = leftMotorBehind ? rightEncoder : leftEncoder;
            float deltaEncoder = leftEncoder - rightEncoder;

            float speedScale = 1.0 - (std::abs(deltaEncoder) / ENCODER_MAX_OFFSET);
            speedScale = std::clamp(speedScale, 0.0, 1.0);

            // Calculate the horizontal speed scaling factor
            float horizontalScale = deltaEncoder *
                                     HORIZONTAL_MULTIPLIER *
                                     std::abs(horizontalSpeed);

            // Drive the motors with the calculated speed
            if (leftMotorBehind)
            {
                leftMotors.moveVoltage(speed - horizontalScale);
                rightMotors.moveVoltage(speed * speedScale + horizontalScale);
            }
            else
            {
                leftMotors.moveVoltage(speed * speedScale - horizontalScale);
                rightMotors.moveVoltage(speed + horizontalScale);
            }
        }

    private:
        static constexpr float HORIZONTAL_DEADZONE = 0.5f;     // deadzone for the joystick
        static constexpr float HORIZONTAL_MULTIPLIER = 0.005f; // %
        static constexpr float HORIZONTAL_ENCODER_SPEED = 2.0f;  // ticks per iteration
        static constexpr float ENCODER_MAX_OFFSET = 100;      // ticks
        static constexpr bool IS_REVERSED = true;              // true if the motors are reversed

        SmartMotorGroup &leftMotors;
        SmartMotorGroup &rightMotors;

        float leftOffset = 0.0;
        float rightOffset = 0.0;
    };
}