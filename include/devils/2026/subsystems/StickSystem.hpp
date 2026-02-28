#pragma once

#include "devils/devils.h"

namespace devils
{
    /**
     * Represents the intake arm and claw system of the robot.
     */
    class StickSystem
    {
    public:
        enum State
        {
            EXTENDED_FAST,
            EXTENDED_SLOW,
            RETRACTED,
            STOP
        };

        StickSystem(
            ADIPneumaticGroup& pto,
            SmartMotorGroup& leftStickMotors,
            SmartMotorGroup& rightStickMotors,
            SmartMotorGroup& leftDriveMotors,
            SmartMotorGroup& rightDriveMotors,
            RotationSensor& stickPositionSensor)
            : pto(pto),
              leftStickMotors(leftStickMotors),
              rightStickMotors(rightStickMotors),
              leftDriveMotors(leftDriveMotors),
              rightDriveMotors(rightDriveMotors),
              stickPositionSensor(stickPositionSensor)
        {
            stickPositionSensor.setPosition(0);
        }

        /**
         * Sets whether the PTO is extended or not.
         * If the PTO is extended, the stick will follow the drive motors.
         * If the PTO is retracted, the stick will move to the position corresponding to the current state.
         * @param extended - True to extend the PTO, false to retract it.
         */
        void setPTOExtended(const bool extended) const
        {
            pto.setExtended(extended);
        }

        /**
         * Sets the current state of the stick system.
         * @param state - The state to set the stick system to. 
         */
        void setState(const State state)
        {
            currentState = state;
        }

        /**
         * Moves the stick based on the current state.
         * If the PTO is extended, the stick will follow the drive motors.
         * Otherwise, it will move to the position corresponding to the current state.
         */
        void moveStick()
        {
            // If the PTO state has changed, start the actuation timer to prevent movement for a short duration
            if (pto.getExtended() != wasPTOExtended)
            {
                ptoActuationTimer.start();
                wasPTOExtended = pto.getExtended();
            }
            if (ptoActuationTimer.getIsRunning())
            {
                leftStickMotors.move(0);
                rightStickMotors.move(0);
                return;
            }

            // If the PTO is extended, the stick should follow the drive motors
            if (pto.getExtended())
            {
                leftStickMotors.move(leftDriveMotors.getLastVoltage());
                rightStickMotors.move(rightDriveMotors.getLastVoltage());
                return;
            }

            // Otherwise, go to the current state
            switch (currentState)
            {
            case EXTENDED_FAST:
                moveToPosition(EXPECTED_POSITION_UP, FAST_SPEED);
                break;
            case EXTENDED_SLOW:
                moveToPosition(EXPECTED_POSITION_UP, SLOW_SPEED);
                break;
            case RETRACTED:
                moveToPosition(EXPECTED_POSITION_DOWN, RETRACTION_SPEED);
                break;
            case STOP:
                moveStick(0);
                break;
            };
        }

        /**
         * Manually moves the stick at a specified speed. The speed should be between -1 and 1, where positive values move the stick up and negative values move the stick down. If the PTO is extended, the stick will not move.
         * @param speed - The speed at which to move the stick (from -1 to 1). Positive values move the stick up, negative values move the stick down. The speed is limited to prevent the stick from moving too fast.
         */
        void moveStick(const float speed)
        {
            if (pto.getExtended()) return; // Don't move stick if PTO is extended

            leftStickMotors.move(speed);
            rightStickMotors.move(speed);
        }

        /**
         * Checks if the stick is stalled during retraction.
         * This is done by checking the current of the stick motors.
         * If the stick is trying to retract but the motor current is above a certain threshold, we can assume it's stalled.
         * @return True if the stick is stalled, false otherwise.
         */
        bool checkStalled() const
        {
            // Check for retraction stall
            const auto stickMotorCurrent = leftStickMotors.getCurrent();
            if (!stickMotorCurrent.isSuccess())
                return false;

            // If the stick is trying to retract but the motor current is above the stall threshold, we can assume it's stalled
            if (currentState == RETRACTED && stickMotorCurrent > RETRACTION_STALL_CURRENT)
                return true;
            return false;
        }

        /**
         * Moves the stick to a target position using a PID controller. The speed of the movement is limited by the `speed` parameter. If the PTO is extended, the stick will not move.
         * @param targetPosition - The target position for the stick to move to (in ticks). This should be set based on the expected positions of the stick when fully extended and fully retracted.
         * @param speed - The maximum speed at which the stick should move (from 0 to 1). This limits the output of the PID controller to prevent the stick from moving too fast.
         */
        void moveToPosition(const float targetPosition, const float speed)
        {
            const auto currentPosition = stickPositionSensor.getAngle();
            if (!currentPosition.isSuccess())
                return;

            const float error = targetPosition - currentPosition; // Calculate the error
            float output = stickPID.update(error); // Calculate the PID output
            if (output > speed)
                output = speed; // Limit the output to the specified speed
            else if (output < -speed)
                output = -speed;

            moveStick(output); // Move the stick based on the PID output
        }

    private:
        static constexpr float FAST_SPEED = 1.0f; // %
        static constexpr float SLOW_SPEED = 0.6f; // %
        static constexpr float RETRACTION_SPEED = 0.6f; // %

        static constexpr float RETRACTION_STALL_CURRENT = 1.8f; // amps
        static constexpr float PTO_PAUSE_DURATION = 0.2f; // seconds

        // Expected position of the stick when fully retracted (in ticks)
        static constexpr float EXPECTED_POSITION_DOWN = 0.0f;
        // Expected position of the stick when fully extended (in ticks)
        static constexpr float EXPECTED_POSITION_UP = -160.0f;

        ADIPneumaticGroup& pto;
        SmartMotorGroup& leftStickMotors;
        SmartMotorGroup& rightStickMotors;

        SmartMotorGroup& leftDriveMotors;
        SmartMotorGroup& rightDriveMotors;

        RotationSensor& stickPositionSensor;

        PIDController stickPID{0.03f, 0.0f, 0.1f}; // PID controller for stick position control

        State currentState{RETRACTED};
        Timer ptoActuationTimer = Timer(PTO_PAUSE_DURATION);

        bool wasPTOExtended = false;
    };
}

