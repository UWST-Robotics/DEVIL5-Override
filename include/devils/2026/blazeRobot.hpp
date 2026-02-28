#pragma once

#include "../devils.h"
#include "./subsystems/intakeSystem.hpp"
#include "./subsystems/StickSystem.hpp"
#include "./subsystems/tubeSystem.hpp"

namespace devils
{
    struct BlazeRobot : Robot
    {
        BlazeRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
            odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            odometry->start();

            toastDisplay->start();
            // devilBotsDisplay->start();

            constexpr auto joystickOptions = ControllerAxis::Options{
                .deadzone = 0.1f, // <-- Minimum input to register
                .startingValue = 0.15f, // <-- Jumps to 15% to overcome motor friction
                .exponent = 3.0f // <-- Cubes the input for finer control at low speeds
            };

            mainController.leftX.setOptions(joystickOptions);
            mainController.leftY.setOptions(joystickOptions);
            mainController.rightX.setOptions(joystickOptions);
            mainController.rightY.setOptions(joystickOptions);

            mainController.up.setMode(ControllerButton::TOGGLED);
            mainController.right.setMode(ControllerButton::TOGGLED);
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
            // MatchAuto::run(chassis, *odometry.get());

            const auto autoBuilder = AutoBuilder(chassis, *odometry.get());
            autoBuilder.rotate(180)->start();

            while (true)
            {
                // Print odom
                auto odom = odometry->getPose();
                Logger::info(odom.toString());
            }
        }

        void opcontrol() override
        {
            // Default State

            // Stop autonomous
            AutoStep::stopAll();

            bool isDriveReversed = false;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                const bool tubeExtendButton = mainController.y; // Tube extend/retract
                const bool intakeArmExtendButton = mainController.l2;
                const bool stickFastButton = mainController.r1;
                const bool stickSlowButton = mainController.r2;

                const bool driveReverseButton = mainController.b; // Reverse controls
                const bool ptoButton = mainController.up; // PTO extend/retract (for testing)

                // Combine Left and Right X Joystick Inputs
                const float combinedX = Math::largestMagnitude({leftX, rightX});

                // Reverse the drive
                if (std::fabsf(leftY) < MAX_ROBOT_SPEED_TO_SWITCH_DIRECTION)
                    isDriveReversed = driveReverseButton;
                const auto driveDirection = isDriveReversed ? -1.0f : 1.0f;

                // Drive the robot with the left joystick
                chassis.move(leftY * driveDirection, combinedX * 0.75f, 0);

                // Intake controls
                intake.runIntake(rightY);
                intake.setArmsExtended(intakeArmExtendButton);
                intake.setStickStalled(stick.checkStalled());

                // Stick pneumatic defaults
                stick.setPTOExtended(ptoButton);

                // For fancy stick controls once testing is done
                if (stickFastButton)
                    stick.setState(StickSystem::State::EXTENDED_FAST);
                else if (stickSlowButton)
                    stick.setState(StickSystem::State::EXTENDED_SLOW);
                else
                    stick.setState(StickSystem::State::RETRACTED);

                stick.moveStick();

                // Hood controls (might not be needed if the hood is a passive system)
                tube.setTubeRaised(!tubeExtendButton);

                // Print odom
                auto odom = odometry->getPose();
                Logger::info(odom.toString());

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
        static constexpr float DEAD_WHEEL_RADIUS = 1;
        static constexpr float MAX_ROBOT_SPEED_TO_SWITCH_DIRECTION = 0.5f;

        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(-1.8, 0);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0, -2.2);

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {10, -9, 8, -7, 6});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-1, 2, -3, 4, -5});

        SmartMotorGroup stickMotorsRight = SmartMotorGroup("StickMotorsRight", {-20});
        SmartMotorGroup stickMotorsLeft = SmartMotorGroup("StickMotorsLeft", {19});

        SmartMotorGroup topRoller = SmartMotorGroup("TopIntakeRoller", {16});
        SmartMotorGroup sideRollers = SmartMotorGroup("SideRollers", {-18, 17});

        ADIPneumatic tubePnematics = ADIPneumatic("TubePneumatics", 'C', false);
        ADIPneumaticGroup intakePnematics = ADIPneumaticGroup("IntakePneumatics", {'D', 'E'}, false);
        ADIPneumaticGroup ptoPnematics = ADIPneumaticGroup("PTOPneumatics", {'A', 'B'}, false);

        RotationSensor stickSensor = RotationSensor("StickSensor", 15);

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 12);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", -11);
        InertialSensor imu = InertialSensor("IMU", 13);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakePnematics, sideRollers, topRoller);
        StickSystem stick = StickSystem(
            ptoPnematics,
            stickMotorsLeft,
            stickMotorsRight,
            leftMotors,
            rightMotors,
            stickSensor);
        TubeSystem tube = TubeSystem(tubePnematics);

        std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
            verticalSensor,
            horizontalSensor,
            DEAD_WHEEL_RADIUS);

        std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();
        // std::shared_ptr<DevilBotsDisplay> devilBotsDisplay = std::make_shared<DevilBotsDisplay>();


        // RobotAutoOptions autoOptions = RobotAutoOptions();
        // std::vector<Routine> routines = {
        //     {0, "Default", false},
        // };
        // // Renderer
        // OptionsRenderer optionsRenderer = OptionsRenderer("Blaze", routines, &autoOptions);
    };
}
