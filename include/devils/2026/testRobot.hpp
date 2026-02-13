#pragma once

#include "../devils.h"
#include "subsystems/armSystem.hpp"

namespace devils
{
    struct TestRobot : Robot
    {
        TestRobot()
        {
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            while (true)
            {
                // Take Controller Inputs
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                //stick movement buttons, will replace later with macros to move stick to preset rotations
                const bool stickScoreButton = mainController.r1; //temp button to move stick
                const bool stickResetButton = mainController.r2; //temp button to move stick

                if (stickScoreButton)
                    armSystem.move(0.7f);
                else if (stickResetButton)
                    armSystem.move(-0.5f);
                else
                    armSystem.move(0.0f);

                floatingRollers.move(rightY);

                // Combine Left and Right X Joystick Inputs
                const float combinedX = JoystickCurve::combine(leftX, rightX);

                // Drive normally
                chassis.move(leftY, combinedX * 0.5f);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
        }

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-11, 12, -13, 14, -15});
        SmartMotorGroup floatingRollers = SmartMotorGroup("FloatingRoller", {-6});
        SmartMotorGroup stick = SmartMotorGroup("Stick", {-4});
        InertialSensor imu = InertialSensor("IMU", 1);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        ArmSystem armSystem = ArmSystem(stick);
        //I made a change
    };
}
