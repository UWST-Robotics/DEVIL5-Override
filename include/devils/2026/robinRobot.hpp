#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a Robin robot and all of its subsystems.
     */
    struct RobinRobot : public Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        RobinRobot()
        {
            imu.setHeadingScale(1.013);

            odometry->useIMU(&imu);
            odometry->setTicksPerRevolution(300);
            odometry->start();
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightX = mainController.get_analog(ANALOG_RIGHT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                //stick movement buttons, will replace later with macros to move stick to preset rotations
                bool stickScoreButton = mainController.get_digital(DIGITAL_R1); //temp button to move stick
                bool stickResetButton = mainController.get_digital(DIGITAL_R2); //temp button to move stick

                if (stickScoreButton) stick.moveVoltage(0.7f);
                else if (stickResetButton) stick.moveVoltage(-0.5f);
                else stick.moveVoltage(0.0f);

                floatingRollers.moveVoltage(rightY);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1, 0.15);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05, 0.2);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.1, 0.2);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1, 0.15);

                rightX *= 0.5;

                // Combine Left and Right X Joystick Inputs
                double combinedX = JoystickCurve::combine(leftX, rightX);

                // Drive normally
                chassis.move(leftY, combinedX * 0.5f);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
            
        }

        void disabled() override
        {
        }

        // VEXBridge
        // VEXBridge bridge = VEXBridge();

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

        // Auto Options
        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Match 1", true},
            {1, "Match 2", true},
            {2, "Skills 1", false},
            {3, "Skills 2", false}};

        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("Robin", routines, &autoOptions);
    };
}