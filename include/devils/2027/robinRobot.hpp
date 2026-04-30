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
         * Creates a new instance of Robin.
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

            //brake modes
            claw.setBrakeMode(MOTOR_BRAKE_HOLD); // Set the claw motors to hold position when no power is applied
            dr4b.setBrakeMode(MOTOR_BRAKE_HOLD); // Set the dr4b motors to hold position when no power is applied
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
                const bool dr4bButtonUp = mainController.r1; //temp button to move stick
                const bool dr4bButtonDown = mainController.r2; //temp button to move stick
                const bool clawButtonOpen = mainController.l1; //temp button to open claw
                const bool clawButtonClose = mainController.l2; //temp button to close claw

                if (dr4bButtonUp) dr4b.move(0.7f);
                else if (dr4bButtonDown) dr4b.move(-0.5f);
                else dr4b.move(0.0f);

                if (clawButtonOpen) claw.move(1.0f);
                else if (clawButtonClose) claw.move(-1.0f);
                else claw.move(0.0f);

                // Combine Left and Right X Joystick Inputs
                const float combinedX = Math::largestMagnitude({leftX, rightX});

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
        SmartMotorGroup dr4b = SmartMotorGroup("DR4B", {-6, 5});
        SmartMotorGroup claw = SmartMotorGroup("Claw", {-4});
        SmartMotorGroup arm = SmartMotorGroup("Arm", {7});
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
