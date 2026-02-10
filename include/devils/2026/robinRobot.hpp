#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a Robin robot and all of its subsystems.
     */
    struct RobinRobot : Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        RobinRobot()
        {
            // Configure Controller Joysticks
            constexpr ControllerAxis::Options joystickOptions = {
                .deadzone = 0.1f, // <-- Minimum input to register
                .startingValue = 0.15f, // <-- Jumps to 15% to overcome motor friction
                .exponent = 3.0f // <-- Cubes the input for finer control at low speeds
            };
            mainController.leftX.setOptions(joystickOptions);
            mainController.leftY.setOptions(joystickOptions);
            mainController.rightX.setOptions(joystickOptions);
            mainController.rightY.setOptions(joystickOptions);

            // Configure IMU scaling
            imu.setHeadingScale(1.013);

            // Configue odometry options
            odometry->useIMU(&imu);
            odometry->setTicksPerRevolution(300);
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            while (true)
            {
                // Take Controller Inputs
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                //stick movement buttons, will replace later with macros to move stick to preset rotations
                const bool stickScoreButton = mainController.r1; //temp button to move stick
                const bool stickResetButton = mainController.r2; //temp button to move stick

                if (stickScoreButton) stick.move(0.7f);
                else if (stickResetButton) stick.move(-0.5f);
                else stick.move(0.0f);

                floatingRollers.move(rightY);

                // Combine Left and Right X Joystick Inputs
                const float combinedX = JoystickCurve::combine(leftX, rightX);

                // Drive normally
                chassis.move(leftY, combinedX * 0.5f, 0);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
        }

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-11, 12, -13, 14, -15});
        SmartMotorGroup floatingRollers = SmartMotorGroup("FloatingRoller", {-6});
        SmartMotorGroup stick = SmartMotorGroup("Stick", {-4});
        InertialSensor imu = InertialSensor("IMU", 1);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);

        // Auto
        std::shared_ptr<TankChassisOdom> odometry = std::make_shared<TankChassisOdom>(chassis, 1.375, 11);

        // // Auto Options
        // RobotAutoOptions autoOptions = RobotAutoOptions();
        // std::vector<Routine> routines = {
        //     {0, "Match 1", true},
        //     {1, "Match 2", true},
        //     {2, "Skills 1", false},
        //     {3, "Skills 2", false}
        // };
        //
        // // Renderer
        // OptionsRenderer optionsRenderer = OptionsRenderer("Robin", routines, &autoOptions);
    };
}
