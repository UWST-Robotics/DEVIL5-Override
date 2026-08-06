#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./autonomous/moaSkillsAutoNew.hpp"

namespace devils
{
    struct PJRobot : Robot
    {
        // Swerve Modules
        SmartMotorGroup frontLeftMotorA = SmartMotorGroup("FrontLeftMotorA", {1});
        SmartMotorGroup frontLeftMotorB = SmartMotorGroup("FrontLeftMotorB", {2});
        SmartMotorGroup frontRightMotorA = SmartMotorGroup("FrontRightMotorA", {10});
        SmartMotorGroup frontRightMotorB = SmartMotorGroup("FrontRightMotorB", {9});
        SmartMotorGroup backLeftMotorA = SmartMotorGroup("BackLeftMotorA", {11});
        SmartMotorGroup backLeftMotorB = SmartMotorGroup("BackLeftMotorB", {12});
        SmartMotorGroup backRightMotorA = SmartMotorGroup("BackRightMotorA", {20});
        SmartMotorGroup backRightMotorB = SmartMotorGroup("BackRightMotorB", {19});

        ADIAnalogInput frontLeftEncoder = ADIAnalogInput("FrontLeftEncoder", 'A', true);
        ADIAnalogInput frontRightEncoder = ADIAnalogInput("FrontRightEncoder", 'B', true);
        ADIAnalogInput backLeftEncoder = ADIAnalogInput("BackLeftEncoder", 'C', true);
        ADIAnalogInput backRightEncoder = ADIAnalogInput("BackRightEncoder", 'D', true);
        
        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, -2.3f);
        SwerveModule frontRightModule = SwerveModule(frontRightMotorA, frontRightMotorB, frontRightEncoder, -0.6f);
        SwerveModule backLeftModule = SwerveModule(backLeftMotorA, backLeftMotorB, backLeftEncoder, 3.4f);
        SwerveModule backRightModule = SwerveModule(backRightMotorA, backRightMotorB, backRightEncoder, 2.6f);
        SwerveChassis swerve = SwerveChassis(
            frontLeftModule, frontRightModule, backLeftModule, backRightModule,
            9.0,
            10.5
        );

        // Subsystems
        // Lift
        SmartMotorGroup liftMotors = SmartMotorGroup("LiftMotors", {3, -8});
        LiftSystem lift = LiftSystem(liftMotors, 21.0f);

        // Claw
        ADIPneumaticGroup clawPiston = ADIPneumaticGroup("ClawPiston", {'F'}, false);
        ClawSystem claw = ClawSystem(clawPiston);

        // Odom
        static constexpr float DEAD_WHEEL_RADIUS = 1;
        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(0.0f, 0.0f);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0.0f, -4.0f);

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", -14);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", -15);
        InertialSensor imu = InertialSensor("IMU", 13);

        std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
            verticalSensor,
            horizontalSensor,
            DEAD_WHEEL_RADIUS);

        // Displays
        //std::shared_ptr<DevilBotsDisplay> devilBotsDisplay = std::make_shared<DevilBotsDisplay>();
        std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();

        // Vexbridge (yikes)
        VEXBridge bridge = VEXBridge();
        VBValue<float> odoXPos = VBValue("X", 0.0f);
        VBValue<float> odoYPos = VBValue("Y", 0.0f);
        VBValue<float> odoHeading = VBValue("Heading", 0.0f);
        

        PJRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
            odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            odometry->start();

            //devilBotsDisplay->start();
            toastDisplay->start();

            constexpr auto joystickOptions = ControllerAxis::Options{
                .deadzone = 0.05f, // <-- Minimum input to register
                .startingValue = 0.05f, // <-- Jumps to 15% to overcome motor friction
                .exponent = 3.0f // <-- Cubes the input for finer control at low speeds
            };
            // Drive controls
            mainController.leftX.setOptions(joystickOptions);
            mainController.leftY.setOptions(joystickOptions);
            mainController.rightX.setOptions(joystickOptions);
            mainController.rightY.setOptions(joystickOptions);
            // Lift controls
            mainController.l1.setMode(ControllerButton::JUST_PRESSED);
            mainController.right.setMode(ControllerButton::JUST_PRESSED);
            mainController.down.setMode(ControllerButton::JUST_PRESSED);
            mainController.y.setMode(ControllerButton::JUST_PRESSED);
            mainController.b.setMode(ControllerButton::JUST_PRESSED);

            lift.calibrateLift();
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
            MoaSkillsAutoNew::run(swerve, *odometry.get(), lift, claw, false);
            //TestingAuto::run(swerve, *odometry.get(), lift, claw);
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
                const float rightX = mainController.rightX * 0.7f;
                // Lift inputs
                bool upOnePinButton = mainController.right;
                bool downOnePinButton = mainController.down;
                bool maxHeightButton = mainController.y;
                bool minHeightButton = mainController.b;
                bool clawToggleButton = mainController.l1;

                // Forward and turn with the left joystick
                // Strafe the robot with the right joystick (Aiden you're a weirdo)
                // Drive the robot
                // Plant if we aren't moving
                if(leftY == 0 && leftX == 0 && rightX == 0){
                    //swerve.plant();
                    swerve.home();
                } else {
                    swerve.move(leftY, leftX, rightX);
                }

                if (upOnePinButton)
                    lift.moveToPosition(lift.convertToPins(lift.getTargetPosition()) + 1); // Move up one pin
                if (downOnePinButton)
                    lift.moveToPosition(lift.convertToPins(lift.getTargetPosition()) - 1); // Move down one pin
                if (maxHeightButton)
                    lift.moveToPosition(25.0f); // Move to max height (in pins)
                if (minHeightButton)
                    lift.moveToPosition(0.0f); // Move to min height (in pins)

                if (clawToggleButton) {
                    claw.setClawClosed(!claw.getClawState()); // Toggle the claw state
                }

                lift.update();

                // Odo telemetry for tuning (vbOdom is curretly broken)
                odoXPos.set(odometry->getPose().x);
                odoYPos.set(odometry->getPose().y);
                odoHeading.set(odometry->getPose().rotation);

                // Delay to prevent the CPU from being overloaded
                pros::delay(10);
            }
        }

        void disabled() override
        {
            // Stop the robot
            swerve.stop();

            // Stop autonomousMain
            AutoStep::stopAll();
        }
    };
}
