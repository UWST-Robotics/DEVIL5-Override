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
         * @param middleIntakeMotors The motors controlling the intake.
         * @param bottomIntakeMotors The motors controlling the intake.
         */
        IntakeSystem(SmartMotorGroup &topIntakeMotors,
                     SmartMotorGroup &middleIntakeMotors,
                     SmartMotorGroup &bottomIntakeMotors)
            :   topIntakeMotors(topIntakeMotors), 
                middleIntakeMotors(middleIntakeMotors),
                bottomIntakeMotors(bottomIntakeMotors)
        {
            topIntakeMotors.setPosition(0);
            middleIntakeMotors.setPosition(0);
            bottomIntakeMotors.setPosition(0);
        }
        
        /**
         * Moves intake using speed
         * @param speed - Voltage to run intake between -1 and 1.
         */
        void intake(float speed)
        {
            speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);

            bottomIntakeMotors.moveVoltage(speed);
            topIntakeMotors.moveVoltage(0);
            middleIntakeMotors.moveVoltage(0);
        }

        /**
        *   Outtakes game pieces to the top goal
        */
        void outtakeTop()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEEDS.topSpeed);
            middleIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEEDS.middleSpeed);
            bottomIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEEDS.bottomSpeed);
        }

        /**
        *   Outtakes game pieces to the middle goal
        */
        void outtakeMid()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_MIDDLE_SPEEDS.topSpeed);
            middleIntakeMotors.moveVoltage(OUTTAKE_MIDDLE_SPEEDS.middleSpeed);
            bottomIntakeMotors.moveVoltage(OUTTAKE_MIDDLE_SPEEDS.bottomSpeed);
        }
    private:
        static constexpr float MAX_SPEED = 1.0;  // %
        static constexpr float MIN_SPEED = -0.6; // %

        static constexpr IntakeMotorSpeeds OUTTAKE_TOP_SPEEDS = {0.8, 0.8, 0.8}; // %
        static constexpr IntakeMotorSpeeds OUTTAKE_MIDDLE_SPEEDS = {0.8, 0.8, 0.8}; // %

        // Hardware
        SmartMotorGroup &topIntakeMotors;
        SmartMotorGroup &middleIntakeMotors;
        SmartMotorGroup &bottomIntakeMotors;
    };
}