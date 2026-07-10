#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./subsystems/intakeSystem.hpp"
#include "./subsystems/StickSystem.hpp"
#include "./subsystems/tubeSystem.hpp"
#include "./subsystems/wingSystem.hpp"

namespace devils
{
    struct PJRobot : Robot
    {
        // Constants
        
        // Hardware
        SmartMotorGroup frontLeftMotorA = SmartMotorGroup("FrontLeftMotorA", {1});
        SmartMotorGroup frontLeftMotorB = SmartMotorGroup("FrontLeftMotorB", {2});
        SmartMotorGroup frontRightMotorA = SmartMotorGroup("FrontRightMotorA", {3});
        SmartMotorGroup frontRightMotorB = SmartMotorGroup("FrontRightMotorB", {4});
        SmartMotorGroup backLeftMotorA = SmartMotorGroup("BackLeftMotorA", {6});
        SmartMotorGroup backLeftMotorB = SmartMotorGroup("BackLeftMotorB", {7});
        SmartMotorGroup backRightMotorA = SmartMotorGroup("BackRightMotorA", {9});
        SmartMotorGroup backRightMotorB = SmartMotorGroup("BackRightMotorB", {8});

        ADIAnalogInput frontLeftEncoder = ADIAnalogInput("FrontLeftEncoder", 'A', false);
        ADIAnalogInput frontRightEncoder = ADIAnalogInput("FrontRightEncoder", 'B', false);
        ADIAnalogInput backLeftEncoder = ADIAnalogInput("BackLeftEncoder", 'D', false);
        ADIAnalogInput backRightEncoder = ADIAnalogInput("BackRightEncoder",'C', false);

        // Drivetrain
        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, -1.4);
        SwerveModule frontRightModule = SwerveModule(frontRightMotorA, frontRightMotorB, frontRightEncoder, 1.3+3.14);
        SwerveModule backLeftModule = SwerveModule(backLeftMotorA, backLeftMotorB, backLeftEncoder, 3.2);
        SwerveModule backRightModule = SwerveModule(backRightMotorA, backRightMotorB, backRightEncoder, -1.57);
        SwerveChassis swerve = SwerveChassis(
            frontLeftModule, frontRightModule, backLeftModule, backRightModule,
            9.0,
            4.5
        );

        // Subsystems
        //InertialSensor imu = InertialSensor("IMU", 13);

        // Displays
        // std::shared_ptr<DevilBotsDisplay> devilBotsDisplay = std::make_shared<DevilBotsDisplay>();
        // AutoPickerDisplay autoPickerDisplay = AutoPickerDisplay(
        //     "PJ Robot",
        //     {
        //         AutoPickerDisplay::Routine{.id = 0, .displayName = "Match Auto", .requiresAllianceColor = false},
        //         AutoPickerDisplay::Routine{.id = 1, .displayName = "Skills Auto", .requiresAllianceColor = false}
        //     });
        //std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();


        // Ok acutal code now
        PJRobot()
        {
            //toastDisplay->start();
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

            mainController.up.setMode(ControllerButton::JUST_PRESSED);
            mainController.right.setMode(ControllerButton::JUST_PRESSED);
        }

        void autonomous() override
        {
            //imu.waitUntilDoneCalibrated();
            //SkillsAuto::run(chassis, *odometry.get(), stick, intake, tube, wings, true);
            //MatchAuto::run(chassis, *odometry.get(), stick, intake, tube, wings, true);
        }

        void opcontrol() override
        {
            // Stop autonomous
            AutoStep::stopAll();

            // Loop
            while (true)
            {
                // Take Controller Inputs
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                // Drive the robot
                swerve.move(leftY, rightX, leftX);

                // Delay to prevent the CPU from being overloaded
                pros::delay(10);
            }
        }

        void disabled() override
        {
            // Stop the robot
            swerve.stop();

            // Stop autonomous
            AutoStep::stopAll();
        }
    };
}
