#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./autonomous/testingAuto.hpp"
#include "./autonomous/moaSkillsAuto.hpp"
#include "./subsystems/liftSystem.hpp"
#include "./subsystems/clawSystem.hpp"

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
            10.5
        );

        // Pneumatics
        ADIPneumaticGroup clawPiston = ADIPneumaticGroup("ClawPiston", {'G'}, false);

        // Motors
        SmartMotorGroup liftMotors = SmartMotorGroup("LiftMotors", {9, -10});

        // Subsystems
        LiftSystem lift = LiftSystem(liftMotors, 31.0f);
        ClawSystem claw = ClawSystem(clawPiston);


        RotationSensor verticalSensor = RotationSensor("VerticalOdom", -12);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 11);
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

        VBValue<float> currentLiftPosition = VBValue("LiftPosition", 0.0f);
        VBValue<float> targetLiftPosition = VBValue("LiftTarget", 0.0f);
        VBValue<float> liftError = VBValue("Lift Error", 0.0f);

        VBValue<float> fcHeading = VBValue("FC Heading", 0.0f);

        VBValue<float> fcStickInX = VBValue("Stick X", 0.0f);
        VBValue<float> fcStickInY = VBValue("Stick Y", 0.0f);

        VBValue<float> fcOutX = VBValue("Out X", 0.0f);
        VBValue<float> fcOutY = VBValue("Out Y", 0.0f);

        VBValue<float> fcHeadingOffset = VBValue("FC Heading offset", 0.0f);
        VBValue<float> fcRotateAmount = VBValue("FC rotate amount", 0.0f);
        

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
            mainController.down.setMode(ControllerButton::JUST_PRESSED);
            mainController.y.setMode(ControllerButton::JUST_PRESSED);
            mainController.b.setMode(ControllerButton::JUST_PRESSED);
            // Field centric button
            mainController.left.setMode(ControllerButton::JUST_PRESSED);

            lift.calibrateLift();
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
            MoaSkillsAuto::run(swerve, *odometry.get(), lift, claw);
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
                // Lift inputs
                bool upOnePinButton = mainController.right;
                bool downOnePinButton = mainController.down;
                bool maxHeightButton = mainController.y;
                bool minHeightButton = mainController.b;
                bool clawToggleButton = mainController.l1;

                bool resetFieldCentricButton = mainController.left;

                float heading = odometry->getPose().rotation;

                // Drive the robot with the left joystick
                // Turn the robot with the right joystick
                // Drive the robot
                // Plant if we aren't moving
                if(leftY == 0 && leftX == 0 && rightX == 0){
                    swerve.plant();
                    //swerve.move(0, 0, 0);
                } else {
                    swerve.move(leftY, rightX, leftX);
                    // swerve.moveFieldCentric(leftY, rightX, leftX, heading);
                }
                // swerve.home();

                if(resetFieldCentricButton){
                    swerve.calibrateFieldCentric();
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

                currentLiftPosition.set(lift.getPosition());
                targetLiftPosition.set(lift.getTargetPosition());
                liftError.set(lift.getError());

                fcHeading.set(heading);

                fcStickInX.set(leftX);
                fcStickInY.set(leftY);

                Vector2 inputVect = Vector2(leftX, leftY);

                fcOutX.set(inputVect.rotate(-heading).x);
                fcOutY.set(inputVect.rotate(-heading).y);

                fcHeadingOffset.set(swerve.fieldCentricHeadingOffset);
                fcRotateAmount.set(-(heading + swerve.fieldCentricHeadingOffset + swerve.constantFCHeadingOffset));

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
