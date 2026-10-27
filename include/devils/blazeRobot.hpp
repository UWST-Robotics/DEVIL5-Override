#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./autonomous/testingAuto.hpp"
#include "./autonomous/moaSkillsAuto.hpp"
#include "./autonomous/moaSkillsAutoNew.hpp"
#include "./subsystems/liftSystem.hpp"
#include "./subsystems/clawSystem.hpp"
#include "./subsystems/wristSystem.hpp"
#include "./subsystems/grabberSystem.hpp"

namespace devils
{
    struct BlazeRobot : Robot
    {
        // Constants
        static constexpr float DEAD_WHEEL_RADIUS = 1;
        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(0.0f, 0.0f);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0.0f, -4.0f);

        // Swerve Modules
        SmartMotorGroup frontLeftMotorA = SmartMotorGroup("FrontLeftMotorA", {1});
        SmartMotorGroup frontLeftMotorB = SmartMotorGroup("FrontLeftMotorB", {2});
        SmartMotorGroup frontRightMotorA = SmartMotorGroup("FrontRightMotorA", {3});
        SmartMotorGroup frontRightMotorB = SmartMotorGroup("FrontRightMotorB", {4});
        SmartMotorGroup backLeftMotorA = SmartMotorGroup("BackLeftMotorA", {5});
        SmartMotorGroup backLeftMotorB = SmartMotorGroup("BackLeftMotorB", {6});
        SmartMotorGroup backRightMotorA = SmartMotorGroup("BackRightMotorA", {7});
        SmartMotorGroup backRightMotorB = SmartMotorGroup("BackRightMotorB", {8});

        ADIAnalogInput frontLeftEncoder = ADIAnalogInput("FrontLeftEncoder", 'A', true);
        ADIAnalogInput frontRightEncoder = ADIAnalogInput("FrontRightEncoder", 'B', true);
        ADIAnalogInput backLeftEncoder = ADIAnalogInput("BackLeftEncoder", 'C', true);
        ADIAnalogInput backRightEncoder = ADIAnalogInput("BackRightEncoder", 'D', true);

        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, 8.1f);
        SwerveModule frontRightModule = SwerveModule(frontRightMotorA, frontRightMotorB, frontRightEncoder, 1.9f);
        SwerveModule backLeftModule = SwerveModule(backLeftMotorA, backLeftMotorB, backLeftEncoder, 4.9f);
        SwerveModule backRightModule = SwerveModule(backRightMotorA, backRightMotorB, backRightEncoder, 4.7f);
        SwerveChassis swerve = SwerveChassis(
            frontLeftModule, frontRightModule, backLeftModule, backRightModule,
            9.0,
            10.5);

        // Pneumatics

        // Motors
        SmartMotorGroup liftMotorsLeft = SmartMotorGroup("LiftMotorsLeft", {9, 10});
        SmartMotorGroup liftMotorsRight = SmartMotorGroup("LiftMotorsRight", {14, 15});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {11, 12, -13});
        SmartMotorGroup grabberMotors = SmartMotorGroup("GrabberMotors", {16});
        SmartMotorGroup wristMotorGroup = SmartMotorGroup("WristMotorGroup", {19});

        // Sensors
        RotationSensor wristSensor = RotationSensor("WristSensor", 14);

        // Subsystems
        LiftSystem lift = LiftSystem(liftMotorsLeft, liftMotorsRight, 18.0f, 0.7874f);
        WristSystem wrist = WristSystem(wristMotorGroup, wristSensor);
        GrabberSystem grabber = GrabberSystem(grabberMotors);

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", -12);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", -11);
        InertialSensor imu = InertialSensor("IMU", 15);

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

        VBValue<float> currentLiftPosition = VBValue("LiftPosition", 0.0f);
        VBValue<float> targetLiftPosition = VBValue("LiftTarget", 0.0f);
        VBValue<float> liftError = VBValue("Lift Error", 0.0f);

        BlazeRobot()
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
            // mainController.down.setMode(ControllerButton::JUST_PRESSED);
            mainController.y.setMode(ControllerButton::JUST_PRESSED);
            mainController.b.setMode(ControllerButton::JUST_PRESSED);
            mainController.a.setMode(ControllerButton::JUST_PRESSED);
            // Field centric button
            // mainController.left.setMode(ControllerButton::JUST_PRESSED);

            wristMotorGroup.setBrakeMode(MOTOR_BRAKE_HOLD);

            lift.calibrateLift();
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
            // MoaSkillsAutoNew::run(swerve, *odometry.get(), lift, claw, true);
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

                // Wrist inputs
                bool wristUp = mainController.up;
                // bool wristNeutral = mainController.left;
                bool wristDown = mainController.down;

                // Grabber inputs
                bool grabberIn = mainController.r1;
                bool grabberOut = mainController.r2;

                bool resetFieldCentricButton = mainController.left;
                bool scoreButton = mainController.a;

                float heading = odometry->getPose().rotation;

                // Drive the robot with the left joystick
                // Turn the robot with the right joystick
                // Drive the robot
                // Plant if we aren't moving
                if (leftY == 0 && leftX == 0 && rightX == 0)
                {
                    swerve.plant();
                    // swerve.move(0, 0, 0);
                    // swerve.home();
                }
                else
                {
                    // swerve.move(leftY, rightX, leftX);
                    swerve.moveFieldCentric(leftY, rightX, leftX, heading);
                }
                // swerve.home();

                if(resetFieldCentricButton){
                    swerve.calibrateFieldCentric();
                }

                if (upOnePinButton)
                    lift.moveToPosition(lift.getPosition(true) + lift.pinsToInches(1)); // Move up one inch
                else if (downOnePinButton)
                    lift.moveToPosition(lift.getPosition(true) - lift.pinsToInches(1)); // Move down one inch
                else if (maxHeightButton)
                    lift.moveToPosition(25.0f); // Move to max height (in pins)
                else if (minHeightButton)
                    lift.moveToPosition(0.0f); // Move to min height (in pins)

                // Fine tweak the lift position with the right joystick
                // lift.globalLiftOffset += rightY * 0.05;

                if (grabberIn)
                    grabber.moveGrabber(100);
                else if (grabberOut)
                    grabber.moveGrabber(-100);
                else
                    grabber.moveGrabber(35);

                // Should more efficiently move the wrist
                if (wristUp)
                    wrist.setTarget(wrist.target++);
                else if (wristDown)
                    wrist.setTarget(wrist.target--);

                intakeMotors.move(rightY);

                // if(wristUp){
                //     wrist.moveUp();
                // }
                // if(wristNeutral){
                //     wrist.moveNeutral();
                // }
                // if(wristDown){
                //     wrist.moveDown();
                // }

                if (scoreButton)
                    score();

                // Moves the wrist to the up position whenever the lift is going up
                if (lift.getTargetPosition() > lift.getAveragePosition())
                    wrist.moveUp();                

                wrist.update();

                lift.update();

                // Odo telemetry for tuning (vbOdom is curretly broken)
                odoXPos.set(odometry->getPose().x);
                odoYPos.set(odometry->getPose().y);
                odoHeading.set(odometry->getPose().rotation);

                currentLiftPosition.set(lift.getPosition(true));
                targetLiftPosition.set(lift.getTargetPosition());
                liftError.set(lift.getError());

                // Delay to prevent the CPU from being overloaded
                pros::delay(10);
            }
        }

        /**
         * Automated scoring routine
         * Moves wrist to neutral
         * Outtakes pin from grabber
         * Moves wrist up
         * Lowers lift
         */
        void score()
        {
            wrist.moveNeutral();
            grabberMotors.move(-100);
            pros::delay(100);
            wrist.moveUp();
            pros::delay(100);
            lift.moveToPosition(0.0f);
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
