#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
//#include "./autonomous/skillsAuto.hpp"
#include "./autonomous/moaSkillsAutoNew.hpp"
#include "./autonomous/scuffedRiceLakeAuto.hpp"
#include "./subsystems/liftSystem.hpp"
#include "./subsystems/clawSystem.hpp"
#include "./subsystems/grabberSystem.hpp"
#include "devilLib/chassis/swerveConstants.hpp"
#include "devilLib/hardware/inertialSensor.hpp"
#include "devils/devilLib/chassis/swerveChassis.hpp"

namespace devils
{
    struct PJRobot : Robot
    {
        // Swerve Modules
        // Brain is front on bare chassis
        SmartMotorGroup frontLeftMotorA = SmartMotorGroup("FrontLeftMotorA", {2});
        SmartMotorGroup frontLeftMotorB = SmartMotorGroup("FrontLeftMotorB", {3});
        SmartMotorGroup frontRightMotorA = SmartMotorGroup("FrontRightMotorA", {18});
        SmartMotorGroup frontRightMotorB = SmartMotorGroup("FrontRightMotorB", {17});
        SmartMotorGroup backLeftMotorA = SmartMotorGroup("BackLeftMotorA", {4});
        SmartMotorGroup backLeftMotorB = SmartMotorGroup("BackLeftMotorB", {1});
        SmartMotorGroup backRightMotorA = SmartMotorGroup("BackRightMotorA", {19});
        SmartMotorGroup backRightMotorB = SmartMotorGroup("BackRightMotorB", {16});

        ADIAnalogInput frontLeftEncoder = ADIAnalogInput("FrontLeftEncoder", 'A', false);
        ADIAnalogInput frontRightEncoder = ADIAnalogInput("FrontRightEncoder", 'H', false);
        ADIAnalogInput backLeftEncoder = ADIAnalogInput("BackLeftEncoder", 'C', false);
        ADIAnalogInput backRightEncoder = ADIAnalogInput("BackRightEncoder", 'D', false);
        
        SwerveConstants swerveConstants = SwerveConstants(1, 9.0, 10.5);
        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, swerveConstants, 0, 1.45);
        SwerveModule frontRightModule = SwerveModule(frontRightMotorA, frontRightMotorB, frontRightEncoder, swerveConstants, 1, 1.575);
        SwerveModule backLeftModule = SwerveModule(backLeftMotorA, backLeftMotorB, backLeftEncoder, swerveConstants, 2, -2.35);
        SwerveModule backRightModule = SwerveModule(backRightMotorA, backRightMotorB, backRightEncoder, swerveConstants, 3, 2.75);
        SwerveChassis swerve = SwerveChassis(
            frontLeftModule, frontRightModule, backLeftModule, backRightModule, swerveConstants
        );

        // Pneumatics
        ADIPneumaticGroup togglesPnematics = ADIPneumaticGroup("TogglePneumatics", {'E'}, true);
        // Motors
        SmartMotorGroup liftMotorsLeft = SmartMotorGroup("LiftMotorsLeft", {6, 7});
        SmartMotorGroup liftMotorsRight = SmartMotorGroup("LiftMotorsRight", {-8, -9});
        SmartMotorGroup topIntakeMotors = SmartMotorGroup("TopIntakeMotors", {15});
        SmartMotorGroup bottomIntakeMotors = SmartMotorGroup("BottomIntakeMotors", {-13});
        SmartMotorGroup throughtakeIntakeMotors = SmartMotorGroup("ThroughtakeIntakeMotors", {11});
        SmartMotorGroup grabberMotors = SmartMotorGroup("GrabberMotors", {10}); 
        SmartMotorGroup wristMotorGroup = SmartMotorGroup("WristMotorGroup", {20});

        // Sensors

        // Subsystems
        LiftSystem lift = LiftSystem(liftMotorsLeft, liftMotorsRight, 27.0f, 0.7874f);
        WristSystem wrist = WristSystem(wristMotorGroup);
        GrabberSystem grabber = GrabberSystem(grabberMotors);
        IntakeSystem intake = IntakeSystem(topIntakeMotors, bottomIntakeMotors, throughtakeIntakeMotors);
        std::shared_ptr<ScoreTask> scorer = std::make_shared<ScoreTask>(wrist, lift, grabber);

        InertialSensor imu = InertialSensor("IMU", 14);

        std::shared_ptr<SwerveOdometry2> odometry = std::make_shared<SwerveOdometry2>(
            swerve,
            swerveConstants);

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
        

        PJRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
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

            // mainController.right.setMode(ControllerButton::JUST_PRESSED);
            mainController.r1.setMode(ControllerButton::TOGGLED);
            mainController.y.setMode(ControllerButton::PRESSED);
            mainController.a.setMode(ControllerButton::PRESSED);
            mainController.b.setMode(ControllerButton::PRESSED);
            mainController.left.setMode(ControllerButton::JUST_PRESSED);
            mainController.r2.setMode(ControllerButton::JUST_PRESSED);
            mainController.l1.setMode(ControllerButton::JUST_PRESSED);
            mainController.l2.setMode(ControllerButton::JUST_PRESSED);
            mainController.down.setMode(ControllerButton::PRESSED);

            lift.calibrateLift();
            scorer->start();
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
            // MoaSkillsAutoNew::run(swerve, *odometry.get(), lift, claw, false);
            //TestingAuto::run(swerve, *odometry.get(), lift, claw);
        }

        void opcontrol() override
        {
            // Stop autonomous
            AutoStep::stopAll();

            // True to not run the outtake or intake
            bool testQuietly = false;
            
            float liftTarget = 0;
            bool extended = false;
            
            // Loop
            while (true)
            {
                // Take Controller Inputs
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.7f;
                // Lift inputs
                bool upOnePinButton = mainController.l1;
                bool downOnePinButton = mainController.l2;
                // bool maxHeightButton = mainController.y;

                bool outtakeButton = mainController.down;
                bool intakeButton = mainController.b;
                bool matchLoadButton = mainController.y;
                bool resetFieldCentricButton = mainController.left;
                bool wristDownButton = mainController.a;
                bool scoreButton = mainController.r2;
                bool toggleButton = mainController.right;
                bool extendButton = mainController.r1;

                // Drive the robot with the left joystick
                // Turn the robot with the right joystick
                // Drive the robot
                // Plant if we aren't moving
                if (leftY == 0 && leftX == 0 && rightX == 0)
                {
                    // swerve.plant();
                    // swerve.move(0, 0, 0);
                    swerve.home();
                }
                else
                {
                    // swerve.move(leftY, rightX, leftX);
                    swerve.moveFieldCentric(leftY, rightX, leftX, imu.getHeading());
                }
                // swerve.home();

                if(resetFieldCentricButton){
                    swerve.calibrateFieldCentric();
                }

                if (scorer->getIsScore() == false)
                {
                    if (upOnePinButton)
                    {
                        if (lift.getTargetPosition() == 0.0f)
                            wrist.setTarget(0);
                        lift.moveToPosition(lift.getTargetPosition() + lift.pinsToInches(1)); // Move up one pin
                        liftTarget = lift.getTargetPosition();
                    }
                    else if (downOnePinButton)
                    {
                        lift.moveToPosition(lift.getTargetPosition() + lift.pinsToInches(-1)); // Move down one pin
                        liftTarget = lift.getTargetPosition();
                    }

                    // Fine tweak the lift position with the right joystick
                    // lift.globalLiftOffset += rightY * 0.05;

                    if (toggleButton)
                    {
                        togglesPnematics.setExtended(!togglesPnematics.getExtended());
                    }

                    if (extendButton && !extended)
                    {
                        lift.moveToPosition(liftTarget);
                        wrist.setTarget(0);
                        extended = true;
                    }
                    else if (!extendButton && extended)
                    {
                        lift.moveToPosition(0.0f);
                        wrist.setTarget(-1);
                        extended = false;
                    }    

                    // uses wrist down state to grab vertical stacks
                    if (wristDownButton && lift.getAveragePosition() < 1.5)
                        wrist.setTarget(0);
                    else if (!wristDownButton && !matchLoadButton && (lift.getAveragePosition() < 1.5f))
                        wrist.setTarget(-1);

                    grabber.moveGrabber(0.75f);

                    // Moves the wrist to the up position whenever the lift is going up
                    // if (lift.getTargetPosition() > lift.getAveragePosition())
                    //     wrist.moveUp();
                }

                // Testing flag to make things nicer while working on other things
                if (!testQuietly) {
                    if (intakeButton)
                    {
                        intake.runIntake(1.0f);
                        throughtakeIntakeMotors.move(0.25f);
                    }
                    else if (outtakeButton)
                    {
                        intake.runIntake(-1.0f);
                        throughtakeIntakeMotors.move(-0.25f);
                    }
                    else if (matchLoadButton)
                    {
                        intake.matchLoad(1.0f);                   
                        throughtakeIntakeMotors.move(0.25f);
                        wrist.setTarget(-1);
                    }
                    else
                    {
                        intake.runIntake(0);                   
                        throughtakeIntakeMotors.move(0.0f);
                    }
                }

                if (scoreButton)
                {
                    scorer->setIsScore(!scorer->getIsScore());
                }

                wrist.update();
                // printf("Wrist target position: %f\n", wrist.getTargetPosition());

                lift.update();

                grabber.update();

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
