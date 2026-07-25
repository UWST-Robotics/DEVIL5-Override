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
        SmartMotorGroup backRightMotorA = SmartMotorGroup("BackRightMotorA", {8});
        SmartMotorGroup backRightMotorB = SmartMotorGroup("BackRightMotorB", {9});

        ADIAnalogInput frontLeftEncoder = ADIAnalogInput("FrontLeftEncoder", 'H', true);
        ADIAnalogInput frontRightEncoder = ADIAnalogInput("FrontRightEncoder", 'B', true);
        ADIAnalogInput backLeftEncoder = ADIAnalogInput("BackLeftEncoder", 'D', true);
        ADIAnalogInput backRightEncoder = ADIAnalogInput("BackRightEncoder",'C', true);

        // Drivetrain
        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, -0.6);
        SwerveModule frontRightModule = SwerveModule(frontRightMotorA, frontRightMotorB, frontRightEncoder, 0.25);
        SwerveModule backLeftModule = SwerveModule(backLeftMotorA, backLeftMotorB, backLeftEncoder, 2.9);
        SwerveModule backRightModule = SwerveModule(backRightMotorA, backRightMotorB, backRightEncoder, 0.8);
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
        std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();

        // Vexbridge (yikes)
        VEXBridge bridge = VEXBridge();
        VBValue<float> targetAngle = VBValue("target angle", 0.0f);
        VBValue<float> currentAngle = VBValue("current angle", 0.0f);
        VBValue<float> error = VBValue("error", 0.0f);


        // Ok acutal code now
        PJRobot()
        {
            toastDisplay->start();
            // devilBotsDisplay->start();

            constexpr auto joystickOptions = ControllerAxis::Options{
                .deadzone = 0.05f, // <-- Minimum input to register
                .startingValue = 0.05f, // <-- Jumps to 15% to overcome motor friction
                .exponent = 1.0f // <-- Cubes the input for finer control at low speeds
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
                // Plant if we aren't moving
                if(leftY == 0 && leftX == 0 && rightX == 0){
                    swerve.plant();
                    //swerve.move(0, 0, 0);
                } else {
                    swerve.move(leftY, rightX, leftX);
                }

                // FL pod debug info
                targetAngle.set(frontLeftModule.getTargetAngle());
                currentAngle.set(frontLeftModule.getPodAngle());
                error.set(frontLeftModule.getStoredError());

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
