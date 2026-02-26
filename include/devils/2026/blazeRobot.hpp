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
            // imu.calibrate();

            // odometry->useIMU(&imu);
            // odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            // odometry->start();

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
        }

        void autonomous() override
        {
            // imu.waitUntilDoneCalibrated();
            // MatchAuto::run(chassis, *odometry.get());
        }

        void opcontrol() override
        {
            // Default State

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

                const bool tubeExtendButton = mainController.right; // Tube extend/retract
                const bool intakeArmExtendButton = mainController.y;
                const bool stickFastButton = mainController.r1;
                const bool stickSlowButton = mainController.r2;
                const bool stickDownButton = mainController.down; // Temporary, should not exist later please

                // Combine Left and Right X Joystick Inputs
                const float combinedX = Math::largestMagnitude({leftX, rightX});

                // Drive normally
                chassis.move(leftY, combinedX * 0.75f, 0);

                // Intake controls
                intake.runIntake(rightY);
                sideIntake.runIntake(rightY * 0.75f);
                intake.setArmsExtended(intakeArmExtendButton);

                // Stick pneumatic defaults
                stickRight.setPTOExtended(true);
                stickLeft.setPTOExtended(true);

                // For fancy stick controls once testing is done
                /*(if (stickFastButton) stick.moveFast();
                else if (stickSlowButton) stick.moveSlow();
                else if (stickFastButton && stickSlowButton) stick.manualRetract();
                //else stick.retract(); */

                if (stickFastButton) 
                {
                    stickRight.manualMove(1); 
                    stickLeft.manualMove(1);
                }
                else if (stickSlowButton)
                {
                    stickRight.manualMove(.5f);
                    stickLeft.manualMove(.5f);
                } 
                else if (stickDownButton) 
                {
                    stickRight.manualMove(-0.35f);
                    stickLeft.manualMove(-0.35f);
                }
                else 
                {
                    stickRight.manualMove(0);
                    stickLeft.manualMove(0);
                }
                // Hood controls (might not be needed if the hood is a passive system)
                tube.setHoodOpen(stickFastButton || stickSlowButton);

                if (tubeExtendButton) tube.setTubeRaised(true);
                else tube.setTubeRaised(false);

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
        // (Copied from PJRobot)
        static constexpr float DEAD_WHEEL_RADIUS = 1;
        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(-0.5, 0);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0, 2.4);

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {10, -9, 8, -7, 6});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-1, 2, -3, 4, -5});

        SmartMotorGroup stickMotorsRight = SmartMotorGroup("StickMotorsRight", {20});
        SmartMotorGroup stickMotorsLeft = SmartMotorGroup("StickMotorsLeft", {-19});

        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {16});
        SmartMotorGroup sideRollers = SmartMotorGroup("SideRollers", {-18, 17});

        ADIPneumatic hoodPneumatics = ADIPneumatic("HoodPneumatics", 'F', true);
        ADIPneumatic tubePnematics = ADIPneumatic("TubePneumatics", 'C', false);
        ADIPneumatic intakePnematicsRight = ADIPneumatic("IntakePneumaticsRight", 'D', false);
        ADIPneumatic intakePnematicsLeft = ADIPneumatic("IntakePneumaticsLeft", 'E', false);
        ADIPneumatic ptoPnematicsLeft = ADIPneumatic("PTOPneumaticsLeft", 'A', true);
        ADIPneumatic ptoPnematicsRight = ADIPneumatic("PTOPneumaticsRight", 'B', true);

        // RotationSensor verticalSensor = RotationSensor("VerticalOdom", 11);
        // RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 12);
        // InertialSensor imu = InertialSensor("IMU", 13);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakePnematicsLeft, intakePnematicsRight, intakeMotors);
        IntakeSystem sideIntake = IntakeSystem(intakePnematicsLeft, intakePnematicsRight, sideRollers);
        StickSystem stickRight = StickSystem(ptoPnematicsRight, stickMotorsRight);
        StickSystem stickLeft = StickSystem(ptoPnematicsLeft, stickMotorsLeft);
        TubeSystem tube = TubeSystem(tubePnematics, hoodPneumatics);

        // std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
        //     verticalSensor,
        //     horizontalSensor,
        //     DEAD_WHEEL_RADIUS);

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
