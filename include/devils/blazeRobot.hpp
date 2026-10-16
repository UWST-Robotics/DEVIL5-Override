#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./subsystems/liftSystem.hpp"
#include "./subsystems/clawSystem.hpp"

namespace devils
{
    struct BlazeRobot : Robot
    {
        // Constants
        static constexpr float DEAD_WHEEL_RADIUS = 1;
        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(-5, 0.5f);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(4, 0);

        // Swerve Modules
        SmartMotorGroup frontLeftMotorA = SmartMotorGroup("FrontLeftMotorA", {1});
        SmartMotorGroup frontLeftMotorB = SmartMotorGroup("FrontLeftMotorB", {2});
        SmartMotorGroup frontRightMotorA = SmartMotorGroup("FrontRightMotorA", {3});
        SmartMotorGroup frontRightMotorB = SmartMotorGroup("FrontRightMotorB", {4});
        SmartMotorGroup backLeftMotorA = SmartMotorGroup("BackLeftMotorA", {5});
        SmartMotorGroup backLeftMotorB = SmartMotorGroup("BackLeftMotorB", {6});
        SmartMotorGroup backRightMotorA = SmartMotorGroup("BackRightMotorA", {7});
        SmartMotorGroup backRightMotorB = SmartMotorGroup("BackRightMotorB", {8});
<<<<<<< Updated upstream
=======

        SmartMotorGroup wristMotorGroup = SmartMotorGroup("WristMotorGroup", {13});
>>>>>>> Stashed changes

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
        ADIPneumaticGroup clawPiston = ADIPneumaticGroup("ClawPiston", {'E'}, false);

        // Motors
        SmartMotorGroup liftMotors = SmartMotorGroup("LiftMotors", {9, -10});

        // Subsystems
<<<<<<< Updated upstream
        LiftSystem lift = LiftSystem(liftMotors, 15.0f);
=======
        LiftSystem lift = LiftSystem(liftMotorLeft, liftMotorRight, 30.0f, 0.3f);
>>>>>>> Stashed changes
        ClawSystem claw = ClawSystem(clawPiston);

<<<<<<< Updated upstream

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 12);
=======
        RotationSensor verticalSensor = RotationSensor("VerticalOdom", -12);
>>>>>>> Stashed changes
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", -11);
        InertialSensor imu = InertialSensor("IMU", 13);

        std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
            verticalSensor,
            horizontalSensor,
            DEAD_WHEEL_RADIUS);

        // Displays
<<<<<<< Updated upstream
        std::shared_ptr<DevilBotsDisplay> devilBotsDisplay = std::make_shared<DevilBotsDisplay>();
=======
        // std::shared_ptr<DevilBotsDisplay> devilBotsDisplay = std::make_shared<DevilBotsDisplay>();
>>>>>>> Stashed changes
        std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();

        // Vexbridge (yikes)
        VEXBridge bridge = VEXBridge();
        VBValue<float> odoXPos = VBValue("X", 0.0f);
        VBValue<float> odoYPos = VBValue("Y", 0.0f);
        VBValue<float> odoHeading = VBValue("Heading", 0.0f);

        VBValue<float> currentLiftPosition = VBValue("LiftPosition", 0.0f);
        VBValue<float> targetLiftPosition = VBValue("LiftTarget", 0.0f);

        BlazeRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
            odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            odometry->start();

<<<<<<< Updated upstream
            devilBotsDisplay->start();
            toastDisplay->start();

            constexpr auto joystickOptions = ControllerAxis::Options{
                .deadzone = 0.1f, // <-- Minimum input to register
                .startingValue = 0.15f, // <-- Jumps to 15% to overcome motor friction
                .exponent = 3.0f // <-- Cubes the input for finer control at low speeds
=======
            // devilBotsDisplay->start();
            toastDisplay->start();

            constexpr auto joystickOptions = ControllerAxis::Options{
                .deadzone = 0.05f,      // <-- Minimum input to register
                .startingValue = 0.05f, // <-- Jumps to 15% to overcome motor friction
                .exponent = 3.0f        // <-- Cubes the input for finer control at low speeds
>>>>>>> Stashed changes
            };

            mainController.leftX.setOptions(joystickOptions);
            mainController.leftY.setOptions(joystickOptions);
            mainController.rightX.setOptions(joystickOptions);
            mainController.rightY.setOptions(joystickOptions);

            mainController.l1.setMode(ControllerButton::TOGGLED);

            lift.calibrateLift();
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
<<<<<<< Updated upstream
            //SkillsAuto::run(swerve, *odometry.get(), stick, intake, tube, wings, false);
            //MatchAuto::run(swerve, *odometry.get(), stick, intake, tube, wings, false);
=======
            MoaSkillsAutoNew::run(swerve, *odometry.get(), lift, claw, true);
            // TestingAuto::run(swerve, *odometry.get(), lift, claw);
>>>>>>> Stashed changes
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

                bool upOnePinButton = mainController.right;
                bool downOnePinButton = mainController.left;
                bool maxHeightButton = mainController.y;
                bool minHeightButton = mainController.b;
                bool clawToggleButton = mainController.l1;

                const float heading = imu.getHeading();

                // Drive the robot with the left joystick
                // Turn the robot with the right joystick
                // Drive the robot
                // Plant if we aren't moving
                if (leftY == 0 && leftX == 0 && rightX == 0)
                {
                    swerve.plant();
                    // swerve.move(0, 0, 0);
                }
                else
                {
                    // swerve.move(leftY, rightX, leftX);
                    swerve.moveFieldCentric(leftY, rightX, leftX, heading);
                }
<<<<<<< Updated upstream
                //swerve.home();

                if (upOnePinButton)
                    lift.moveToPosition(lift.getPosition() + 1); // Move up one pin
                else if (downOnePinButton)
                    lift.moveToPosition(lift.getPosition() - 1); // Move down one pin
                else if (maxHeightButton)
                    lift.moveToPosition(25.0f); // Move to max height (in pins)
                else if (minHeightButton)
                    lift.moveToPosition(0.0f); // Move to min height (in pins)

                if (clawToggleButton) {
                    claw.setClawClosed(clawToggleButton); // Toggle the claw state
                }

=======
                // swerve.home();

                if (resetFieldCentricButton)
                {
                    swerve.calibrateFieldCentric();
                }

                if (upOnePinButton)
                    lift.moveToPosition(lift.getTargetPosition() + lift.pinsToInches(0.5)); // Move up one pin
                if (downOnePinButton)
                    lift.moveToPosition(lift.getTargetPosition() - lift.pinsToInches(0.5)); // Move down one pin
                if (maxHeightButton)
                    lift.moveToPosition(lift.getMaxHeight()); // Move to max height (in inches)
                if (minHeightButton)
                    lift.moveToPosition(0.0f); // Move to min height (in inches)

                // Fine tweak the lift position with the right joystick
                // lift.globalLiftOffset += rightY * 0.05;

                if (clawToggleButton)
                {
                    claw.setClawClosed(!claw.getClawState()); // Toggle the claw state
                }

                if (wristUp)
                {
                    wristMotorGroup.move(.5f);
                    wrist.moveUp();
                    printf("fuckkkkkkk");
                    Logger::info("Wrist up");
                }
                else
                {
                    wristMotorGroup.move(0);
                }
                if (wristNeutral)
                {
                    wrist.moveNeutral();
                    Logger::info("Wrist neutral");
                }
                if (wristDown)
                {
                    wrist.moveDown();
                    Logger::info("Wrist down");
                }

                wrist.update();

                lift.update();

>>>>>>> Stashed changes
                // Odo telemetry for tuning (vbOdom is curretly broken)
                odoXPos.set(odometry->getPose().x);
                odoYPos.set(odometry->getPose().y);
                odoHeading.set(odometry->getPose().rotation);

                currentLiftPosition.set(lift.getPosition());
                targetLiftPosition.set(lift.getTargetPosition());

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
