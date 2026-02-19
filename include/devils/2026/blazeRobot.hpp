#pragma once

#include "../devils.h"
#include "./subsystems/intakeSystem.hpp"
#include "./subsystems/StickSystem.hpp"
#include "./subsystems/tubeSystem.hpp"
// #include "./autonomous/blazeSkillsAuto.hpp"

namespace devils
{
    struct BlazeRobot : Robot
    {
        BlazeRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
            odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            odometry->start();

            toastDisplay->start();

            auto joystickOptions = ControllerAxis::Options{
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
            imu.waitUntilDoneCalibrated();
            // BlazeSkillsAuto::run(chassis, *odometry.get(), intake);
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

                const bool hoodExtendButton = mainController.l1; // hood extend/retract
                const bool tubeExtendButton = mainController.right; // tube extend/retract
                const bool intakeArmExtendButton = mainController.y;
                const bool stickFastButton = mainController.r1;
                const bool stickSlowButton = mainController.r2;

                // Combine Left and Right X Joystick Inputs
                const float combinedX = Math::largestMagnitude({leftX, rightX});


                // Drive normally
                chassis.move(leftY, combinedX * 0.5f, 0);

                //Intake shit
                intake.runIntake(rightY);
                intake.setArmsExtended(intakeArmExtendButton);

                tube.setHoodOpen(stickFastButton || stickSlowButton);

                if (stickFastButton) stick.moveFast();
                else if (stickSlowButton) stick.moveSlow();
                else stick.retract();

                if (tubeExtendButton) tube.setTubeRaised(true);
                else tube.setTubeRaised(false);

                stick.setPTOExtended(false);


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
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-11, 12, -13, 14, -15});

        SmartMotorGroup stickMotorsRight = SmartMotorGroup("StickMotorsRight", {-4});
        SmartMotorGroup stickMotorsLeft = SmartMotorGroup("StickMotorsLeft", {6});

        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {-7, 8, -9});

        ADIPneumatic hoodPneumatics = ADIPneumatic("HoodPneumatics", 'H', true);
        ADIPneumatic tubePnematics = ADIPneumatic("TubePneumatics", 'G', true);
        ADIPneumatic intakePnematics = ADIPneumatic("IntakePneumatics", 'F', true);
        ADIPneumatic ptoPnematics = ADIPneumatic("PTOPneumatics", 'E', true);

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 9);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 8);
        InertialSensor imu = InertialSensor("IMU", 4);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakePnematics, intakeMotors);
        StickSystem stick = StickSystem(ptoPnematics, stickMotorsRight);
        TubeSystem tube = TubeSystem(tubePnematics, hoodPneumatics);

        std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
            verticalSensor,
            horizontalSensor,
            DEAD_WHEEL_RADIUS);

        std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();


        // RobotAutoOptions autoOptions = RobotAutoOptions();
        // std::vector<Routine> routines = {
        //     {0, "Default", false},
        // };
        // // Renderer
        // OptionsRenderer optionsRenderer = OptionsRenderer("Blaze", routines, &autoOptions);
    };
}
