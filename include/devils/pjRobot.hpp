#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./autonomous/moaSkillsAutoNew.hpp"
#include "./subsystems/liftSystem.hpp"
#include "./subsystems/clawSystem.hpp"
#include "./subsystems/grabberSystem.hpp"

namespace devils
{
    struct PJRobot : Robot
    {
        // Swerve Modules
        SmartMotorGroup frontLeftMotorA = SmartMotorGroup("FrontLeftMotorA", {20});
        SmartMotorGroup frontLeftMotorB = SmartMotorGroup("FrontLeftMotorB", {19});
        SmartMotorGroup frontRightMotorA = SmartMotorGroup("FrontRightMotorA", {11});
        SmartMotorGroup frontRightMotorB = SmartMotorGroup("FrontRightMotorB", {12});
        SmartMotorGroup backLeftMotorA = SmartMotorGroup("BackLeftMotorA", {10});
        SmartMotorGroup backLeftMotorB = SmartMotorGroup("BackLeftMotorB", {9});
        SmartMotorGroup backRightMotorA = SmartMotorGroup("BackRightMotorA", {1});
        SmartMotorGroup backRightMotorB = SmartMotorGroup("BackRightMotorB", {2});

        ADIAnalogInput frontLeftEncoder = ADIAnalogInput("FrontLeftEncoder", 'D', true);
        ADIAnalogInput frontRightEncoder = ADIAnalogInput("FrontRightEncoder", 'B', true);
        ADIAnalogInput backLeftEncoder = ADIAnalogInput("BackLeftEncoder", 'C', true);
        ADIAnalogInput backRightEncoder = ADIAnalogInput("BackRightEncoder", 'A', true);
        
        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, -1.6f);
        SwerveModule frontRightModule = SwerveModule(frontRightMotorA, frontRightMotorB, frontRightEncoder, 1.5f);
        SwerveModule backLeftModule = SwerveModule(backLeftMotorA, backLeftMotorB, backLeftEncoder, 0.6f);
        SwerveModule backRightModule = SwerveModule(backRightMotorA, backRightMotorB, backRightEncoder, -0.3f);
        SwerveChassis swerve = SwerveChassis(
            frontLeftModule, frontRightModule, backLeftModule, backRightModule,
            9.0,
            10.5
        );

        // Subsystems
        // Lift
        SmartMotorGroup liftMotorsLeft = SmartMotorGroup("LiftMotorsLeft", {8, 7});
        SmartMotorGroup liftMotorsRight = SmartMotorGroup("LiftMotorsLeft", {12, 14});

        // Grabber
        SmartMotorGroup rollers = SmartMotorGroup("Rollers", {4, -5});

        // Subsystems
        LiftSystem lift = LiftSystem(liftMotorsLeft, liftMotorsRight, 21.0f, 0.7874f);

        // Grabber
        GrabberSystem grabber = GrabberSystem(rollers);

        // Claw
        ADIPneumaticGroup clawPiston = ADIPneumaticGroup("ClawPiston", {'F'}, false);
        ClawSystem claw = ClawSystem(clawPiston);

        // Odom
        static constexpr float DEAD_WHEEL_RADIUS = 1;
        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(0.0f, 0.0f);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0.0f, -4.0f);

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", -17);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", -16);
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
                const float rightY = mainController.rightY;
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

                // Grabber
                grabber.moveGrabber(rightY);

                 if (upOnePinButton)
                    lift.moveToPosition(lift.getTargetPosition() + lift.pinsToInches(1)); // Move up one pin
                if (downOnePinButton)
                    lift.moveToPosition(lift.getTargetPosition() - lift.pinsToInches(1)); // Move down one pin
                if (maxHeightButton)
                    lift.moveToPosition(lift.getMaxHeight()); // Move to max height (in inches)
                if (minHeightButton)
                    lift.moveToPosition(0.0f); // Move to min height (in inches)

                // Fine tweak the lift position with the right joystick
                // lift.globalLiftOffset += rightY * 0.03;

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
