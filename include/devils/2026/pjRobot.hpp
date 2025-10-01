#pragma once

#include "../devils.h"
#include "subsystems/intakeSystem.hpp"
#include "pros/adi.hpp"

namespace devils
{
    struct PJRobot : public Robot
    {
        PJRobot()
        {

        }

        void autonomous() override
        {
            
        }

        void opcontrol() override
        {
            // Default State

            // Stop autonomous
            AutoStep::stopAll();

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightX = mainController.get_analog(ANALOG_RIGHT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1, 0.15);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05, 0.2);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.1, 0.2);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1, 0.15, 0.8);

                // Decrease turning speed for improved control
                rightX *= 0.5;

                // Combine Left and Right X Joystick Inputs
                double combinedX = JoystickCurve::combine(leftX, rightX);

                intake.move(rightY);

                // Drive normally
                chassis.move(leftY, combinedX);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();

            // Stop autonomous
            AutoStep::stopAll();
        }

        // Constants
        static constexpr double DEAD_WHEEL_RADIUS = 0.991; // in (slightly smaller to account for roller play)
        static constexpr double CONVEYOR_LENGTH = 84.0;    // teeth
        static constexpr double HOOK_INTERVAL = 21.0;      // teeth
        static constexpr double REJECT_OFFSET = 13;        // teeth

        Vector2 verticalSensorOffset = Vector2(-0.5, 0);
        Vector2 horizontalSensorOffset = Vector2(0, 1);

        // Hardware
        // VEXBridge bridge = VEXBridge();

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-11, 3, -12, 4, -1});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {6, -9, 5, -10, 21});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {1, 2});


        // LED Strips
        LEDStrip ledStrip = LEDStrip(9);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakeMotors);

        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Default", false},
        };
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("PepperJack", routines, &autoOptions);
    };
}