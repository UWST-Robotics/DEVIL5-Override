#pragma once

#include "./devils.h"
#include "./subsystems/clawSystem.hpp"
#include "./subsystems/armSystem.hpp"

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

            mainController.l1.setMode(ControllerButton::TOGGLED);
            mainController.l2.setMode(ControllerButton::TOGGLED);
            mainController.x.setMode(ControllerButton::TOGGLED);
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

                const bool liftUpButton = mainController.r1; //temp button to move lift
                const bool liftDownButton = mainController.r2; //temp button to move lift
                const bool clawButton = mainController.l1; //temp button to open/close claw

                intake.move(rightY * 0.8f);

                claw.setClawClosed(clawButton);

                if (liftUpButton)
                    arm.moveLift(0.5f);
                else if (liftDownButton)
                    arm.moveLift(-0.3f);
                else
                    arm.stopLift();

                // Drive normally
                chassis.move(leftY, leftX * 0.5f, 0);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
        }

        // Hardware
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {18, -17});
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-14, 15});
        SmartMotorGroup intake = SmartMotorGroup("Intake", {-8, -12}); //Main intake first, floating roller second (reversed)
        SmartMotorGroup lift = SmartMotorGroup("Lift", {9}); //Main lift control
        SmartMotorGroup chainBar = SmartMotorGroup("ChainBar", {10}); //Chainbar at the end of the lift
        ADIPneumaticGroup clawPneumatics = ADIPneumaticGroup("ClawPneumatics", {'A'}, false);
        InertialSensor imu = InertialSensor("IMU", 1);


        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        ClawSystem claw = ClawSystem(clawPneumatics); // Claw flip and door pneumatics are not used in this robot
        ArmSystem arm = ArmSystem(lift, chainBar); // Arm system for lift and chain bar control

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
