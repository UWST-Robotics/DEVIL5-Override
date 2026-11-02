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
#include "./subsystems/intakeSystem.hpp"
#include "./subsystems/scoreTask.hpp"
#include "devilLib/chassis/swerveConstants.hpp"

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
        
        SwerveConstants swerveConstants = SwerveConstants(0, 9.0, 10.5);
        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, swerveConstants, 0);
        SwerveModule frontRightModule = SwerveModule(frontRightMotorA, frontRightMotorB, frontRightEncoder, swerveConstants, 1);
        SwerveModule backLeftModule = SwerveModule(backLeftMotorA, backLeftMotorB, backLeftEncoder, swerveConstants, 2);
        SwerveModule backRightModule = SwerveModule(backRightMotorA, backRightMotorB, backRightEncoder, swerveConstants, 3);
        SwerveChassis swerve = SwerveChassis(
            frontLeftModule, frontRightModule, backLeftModule, backRightModule, swerveConstants
        );

        // Pneumatics

        // Motors
        SmartMotorGroup liftMotorsLeft = SmartMotorGroup("LiftMotorsLeft", {9, 10});
        SmartMotorGroup liftMotorsRight = SmartMotorGroup("LiftMotorsRight", {-14, -15});
        SmartMotorGroup topIntakeMotors = SmartMotorGroup("TopIntakeMotors", {11});
        SmartMotorGroup bottomIntakeMotors = SmartMotorGroup("BottomIntakeMotors", {-13});
        SmartMotorGroup throughtakeIntakeMotors = SmartMotorGroup("ThroughtakeIntakeMotors", {12});
        SmartMotorGroup grabberMotors = SmartMotorGroup("GrabberMotors", {16});
        SmartMotorGroup wristMotorGroup = SmartMotorGroup("WristMotorGroup", {19});

        // Sensors

        // Subsystems
        LiftSystem lift = LiftSystem(liftMotorsLeft, liftMotorsRight, 27.0f, 0.7874f);
        WristSystem wrist = WristSystem(wristMotorGroup);
        GrabberSystem grabber = GrabberSystem(grabberMotors);
        IntakeSystem intake = IntakeSystem(topIntakeMotors, bottomIntakeMotors, throughtakeIntakeMotors);
        std::shared_ptr<ScoreTask> scorer = std::make_shared<ScoreTask>(wrist, lift, grabber);

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", -18);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", -19);
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
            mainController.l2.setMode(ControllerButton::JUST_PRESSED);

            mainController.left.setMode(ControllerButton::JUST_PRESSED);
            mainController.down.setMode(ControllerButton::JUST_PRESSED);
            // mainController.up.setMode(ControllerButton::JUST_PRESSED);

            mainController.r1.setMode(ControllerButton::JUST_PRESSED);
            mainController.r2.setMode(ControllerButton::JUST_PRESSED);
            mainController.b.setMode(ControllerButton::JUST_PRESSED);
            mainController.x.setMode(ControllerButton::JUST_PRESSED);

            // Field centric button
            // mainController.left.setMode(ControllerButton::JUST_PRESSED);

            // wristMotorGroup.setBrakeMode(MOTOR_BRAKE_HOLD);

            lift.calibrateLift();
            scorer->start();
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
                bool upOnePinButton = mainController.r1;
                bool downOnePinButton = mainController.r2;
                bool maxHeightButton = mainController.x;
                bool minHeightButton = mainController.b;

                // grabber inputs
                bool grabberOutButton = mainController.up;

                // Wrist inputs
                bool wristUp = mainController.l1;
                bool wristDown = mainController.l2;

                bool resetFieldCentricButton = mainController.left;
                bool scoreButton = mainController.a;

                bool intakeInButton = mainController.y;
                bool intakeOutButton = mainController.right;

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

                if (scorer->getIsScore() == false)
                {
                    if (upOnePinButton)
                        lift.moveToPosition(lift.getTargetPosition() + lift.pinsToInches(1)); // Move up one pin
                    else if (downOnePinButton)
                        lift.moveToPosition(lift.getTargetPosition() + lift.pinsToInches(-1)); // Move down one pin
                    else if (maxHeightButton)
                        lift.moveToPosition(lift.getMaxHeight()); // Move to max height (in pins)
                    else if (minHeightButton)
                        lift.moveToPosition(0.0f); // Move to min height (in pins)

                    // Fine tweak the lift position with the right joystick
                    // lift.globalLiftOffset += rightY * 0.05;

                    // run grabber
                    if (grabberOutButton)
                        grabber.moveGrabber(-1);
                    else
                        grabber.moveGrabber(0.75f);

                    // Should more efficiently move the wrist
                    if (wristUp)
                    {
                        wrist.setTarget(wrist.targetState + 1); 
                        // prevent objects from clipping when placed in grabber by intake
                        if (lift.getTargetPosition() <= 1.0f)
                        {
                            if (wrist.getTargetState() > -1)
                            {
                                printf("wrist position: %f\n", wrist.getTargetState());
                                wrist.setTarget(0);
                                lift.moveToPosition(lift.pinsToInches(1));
                            }
                        }
                    }
                    else if (wristDown)
                        wrist.setTarget(wrist.targetState - 1);


                    // Moves the wrist to the up position whenever the lift is going up
                    // if (lift.getTargetPosition() > lift.getAveragePosition())
                    //     wrist.moveUp();
                }

                intake.matchLoad(abs(rightY));

                if (intakeInButton)
                {
                    intake.runIntake(1.0f);
                }
                else if (intakeOutButton)
                {
                    intake.runIntake(-1.0f);
                }
                else
                {
                    intake.runIntake(0.0f);
                    throughtakeIntakeMotors.move(0.5f);
                }

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
                {
                    // scorer.setState(ScoreSystem::State::SCORING);
                    scorer->setIsScore(!scorer->getIsScore());
                }

                wrist.update();
                // printf("Wrist target position: %f\n", wrist.getTargetPosition());

                lift.update();

                // Odo telemetry for tuning (vbOdom is curretly broken)
                odoXPos.set(odometry->getPose().x);
                odoYPos.set(odometry->getPose().y);
                odoHeading.set(odometry->getPose().rotation);

                currentLiftPosition.set(lift.getAveragePosition());
                targetLiftPosition.set(lift.getTargetPosition());
                liftError.set(lift.getError());

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
