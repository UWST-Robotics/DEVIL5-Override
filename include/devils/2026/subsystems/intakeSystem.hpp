#pragma once

#include "devils/devils.h"
#include <algorithm>

namespace devils
{
    /// @brief Struct to hold motor speeds for the intake system.
    struct IntakeMotorSpeeds
    {
        float topSpeed;
        float middleSpeed;
        float bottomSpeed;
    };

    /**
     * Represents the intake arm and claw system of the robot.
     */
    class IntakeSystem
    {
    public:
        /**
         * Creates a new instance of the intake system.
         * @param topIntakeMotors The motors controlling the intake.
         * @param bottomIntakeMotors The motors controlling the intake.
         */
        IntakeSystem(SmartMotorGroup &topIntakeMotors,
                     SmartMotorGroup &bottomIntakeMotors,
                     SmartMotorGroup &backIntakeMotors)
            :   topIntakeMotors(topIntakeMotors), 
                bottomIntakeMotors(bottomIntakeMotors),
                backIntakeMotors(backIntakeMotors)
        {
            topIntakeMotors.setPosition(0);
            bottomIntakeMotors.setPosition(0);
            backIntakeMotors.setPosition(0);
        }

        /**
         * Moves intake using speed
         * @param speed - Voltage to run intake between -1 and 1.
         */
        void defaultIntake(float speed)
        {
            speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);

            bottomIntakeMotors.moveVoltage(speed);
            backIntakeMotors.moveVoltage(speed);
            topIntakeMotors.moveVoltage(speed);
        }

        /**
         *   Outtakes game pieces to the top goal
         */
        void outtakeTop()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            backIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            bottomIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
        }

        /**
         *   Stops outtaking game pieces
         */
        void stopOuttake()
        {
            topIntakeMotors.stop();
        }

        /**
         *   Outtakes game pieces to the middle goal
         */
        void outtakeMid()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_MIDDLE_SPEED);
            backIntakeMotors.moveVoltage(-OUTTAKE_MIDDLE_SPEED);
            bottomIntakeMotors.moveVoltage(-OUTTAKE_MIDDLE_SPEED);
        }

    private:
        static constexpr float MAX_SPEED = 1.0;  // %
        static constexpr float MIN_SPEED = -0.6; // %

        static constexpr float OUTTAKE_TOP_SPEED = 0.8;     // %
        static constexpr float OUTTAKE_MIDDLE_SPEED = -0.5; // %

        // Hardware
        SmartMotorGroup &topIntakeMotors;
        SmartMotorGroup &bottomIntakeMotors;
        SmartMotorGroup &backIntakeMotors;
    };
}