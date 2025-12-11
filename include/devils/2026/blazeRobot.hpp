#pragma once

#include "../devils.h"
#include "subsystems/intakeSystem.hpp"
#include "./autonomous/blazeSkillsAuto.hpp"
#include "pros/adi.hpp"

namespace devils
{
    struct BlazeRobot : public Robot
    {
        BlazeRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
            odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            odometry->start();
        }

        virtual void autonomous() override
        {
            imu.waitUntilCalibrated();
            BlazeSkillsAuto::run(chassis, *odometry.get(), intake);
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

                // Run Cyclers
                if (exitCyclerButton)
                    intake.setIntakeMode(IntakeMode::SideGoal); // Score Top
                else if (midOuttakeButton)
                    intake.setIntakeMode(IntakeMode::MidBottom); // Score Bottom
                else if (exitCyclerMidButton)
                    intake.setIntakeMode(IntakeMode::MidTop); // Score Mid

                bool isScoring = exitCyclerButton || midOuttakeButton || exitCyclerMidButton;

                intake.runIntake(isScoring ? 1.0f : rightY);
                intake.setArmsExtended(intakeExtendButton);
                intake.setHoodExtended(hoodExtendButton);

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

        // Constants
        // (Copied from PJRobot)
        static constexpr double DEAD_WHEEL_RADIUS = 1;
        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(0, 0);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0, 2.34);

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-11, 12, -13, 14, -15});

        SmartMotorGroup frontBottomIntakeMotors = SmartMotorGroup("FrontBottomIntake", {-1});
        SmartMotorGroup frontTopIntakeMotors = SmartMotorGroup("FrontTopIntake", {-10});
        SmartMotorGroup frontIntakeRollers = SmartMotorGroup("FrontIntakeRollers", {2, -3});
        SmartMotorGroup backIntakeMotors = SmartMotorGroup("BackIntakeMotors", {-6});
        SmartMotorGroup cyclerMotors = SmartMotorGroup("CyclerMotors", {5});

        ADIPneumatic intakePneumaticsLeft = ADIPneumatic("IntakePneumatics", 1);
        ADIPneumatic intakePneumaticsRight = ADIPneumatic("IntakePneumatics", 2);
        ADIPneumatic hoodPneumatics = ADIPneumatic("HoodPneumatics", -8);

        OpticalSensor colorSensor = OpticalSensor("InventoryColorSensor", 7);
        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 9);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 8);
        InertialSensor imu = InertialSensor("IMU", 4);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(
            frontTopIntakeMotors,
            frontBottomIntakeMotors,
            frontIntakeRollers,
            backIntakeMotors,
            cyclerMotors,
            colorSensor,
            intakePneumaticsLeft,
            intakePneumaticsRight,
            hoodPneumatics);

        std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
            verticalSensor,
            horizontalSensor,
            DEAD_WHEEL_RADIUS);

        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Default", false},
        };
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("Blaze", routines, &autoOptions);
    };
}