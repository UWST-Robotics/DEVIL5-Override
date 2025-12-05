#pragma once

#include "../devils.h"
#include "subsystems/intakeSystem.hpp"
#include "pros/adi.hpp"

namespace devils
{
    struct BlazeRobot : public Robot
    {
        BlazeRobot()
        {
        }

        virtual void autonomous() override
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

                bool exitCyclerButton = mainController.get_digital(DIGITAL_R1);    // high goal from basket
                bool midOuttakeButton = mainController.get_digital(DIGITAL_L2);    // mid goal from ground
                bool intakeExtendButton = mainController.get_digital(DIGITAL_Y);   // intake extend/retract
                bool hoodExtendButton = mainController.get_digital(DIGITAL_L1);    // hood extend/retract
                bool exitCyclerMidButton = mainController.get_digital(DIGITAL_R2); // mid goal from cycler

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1, 0.15);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05, 0.2);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.1, 0.2);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1, 0.15);

                // Decrease turning speed for improved control
                rightX *= 0.5;

                // Combine Left and Right X Joystick Inputs
                double combinedX = JoystickCurve::combine(leftX, rightX);

                // Run Intake/Outtake
                if (exitCyclerButton)
                    intake.exitCycler();
                else if (midOuttakeButton)
                    intake.outtakeMid();

                else if (exitCyclerMidButton)
                    intake.exitCyclerMid();
                else
                    intake.defaultIntake(rightY);

                if (intakeExtendButton)
                    intake.intakeExtend();
                else
                    intake.intakeRetract();

                if (hoodExtendButton)
                    intake.hoodExtend();
                else
                    intake.hoodRetract();

                // Drive normally
                chassis.move(leftY, combinedX * 0.5f);

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

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-11, 12, -13, 14, -15});
        SmartMotorGroup topIntakeMotors = SmartMotorGroup("TopIntakeMotors", {-1});
        SmartMotorGroup bottomIntakeMotors = SmartMotorGroup("BottomIntakeMotors", {-10, 2, -3});
        SmartMotorGroup backIntakeMotors = SmartMotorGroup("BackIntakeMotors", {-6});
        SmartMotorGroup cyclerMotors = SmartMotorGroup("CyclerMotors", {-5});

        ADIPneumatic intakePneumaticsLeft = ADIPneumatic("IntakePneumatics", 1);
        ADIPneumatic intakePneumaticsRight = ADIPneumatic("IntakePneumatics", 2);
        ADIPneumatic hoodPneumatics = ADIPneumatic("HoodPneumatics", -8);

        OpticalSensor colorSensor = OpticalSensor("InventoryColorSensor", 4);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(
            topIntakeMotors,
            bottomIntakeMotors,
            backIntakeMotors,
            cyclerMotors,
            colorSensor,
            intakePneumaticsLeft,
            intakePneumaticsRight,
            hoodPneumatics);

        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Default", false},
        };
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("Blaze", routines, &autoOptions);
    };
}