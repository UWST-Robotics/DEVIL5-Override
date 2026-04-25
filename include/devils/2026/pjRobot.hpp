#pragma once

#include "../devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./subsystems/intakeSystem.hpp"
#include "./subsystems/StickSystem.hpp"
#include "./subsystems/tubeSystem.hpp"
#include "./subsystems/wingSystem.hpp"

namespace devils
{
    struct PJRobot : Robot
    {
        //for rising edge detection
        bool prevTubeInput = false;

        PJRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
            odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            odometry->start();

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

            mainController.up.setMode(ControllerButton::JUST_PRESSED);
            mainController.right.setMode(ControllerButton::JUST_PRESSED);
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
            //SkillsAuto::run(chassis, *odometry.get(), stick, intake, tube, wings, true);
            MatchAuto::run(chassis, *odometry.get(), stick, intake, tube, wings, true);
        }

        void opcontrol() override
        {
            // Home the stick
            stick.homeStick();

            // Stop autonomous
            AutoStep::stopAll();

            bool isDriveReversed = false;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                const bool tubeExtendButton = mainController.l2; // Tube extend/retract
                const bool intakeArmExtendButton = mainController.y;
                const bool wingExtendButton = mainController.x; // Wing raise/lower
                const bool stickFastButton = mainController.r1;
                const bool stickSlowButton = mainController.r2;

                const bool driveReverseButton = mainController.b; // Reverse controls
                const bool ptoButton = mainController.right; // PTO extend/retract (for testing)

                // Combine Left and Right X Joystick Inputs
                const float combinedX = Math::largestMagnitude({leftX, rightX});

                // Reverse the drive
                if (std::abs(leftY) < MAX_ROBOT_SPEED_TO_SWITCH_DIRECTION)
                    isDriveReversed = driveReverseButton;
                const auto driveDirection = isDriveReversed ? -1.0f : 1.0f;

                // Drive the robot with the left joystick
                chassis.move(leftY * driveDirection, combinedX * 0.75f, 0);

                // Intake controls
                intake.runIntake(rightY);
                intake.setArmsExtended(intakeArmExtendButton);
                intake.setStickStalled(stick.checkStalled());
                stick.rumbleIfStalled(mainController);

                // Stick pneumatic defaults
                if (ptoButton)
                    stick.setPTOExtended(!stick.getPTOExtended());

                // For fancy stick controls once testing is done
                if (stickFastButton)
                    stick.setState(StickSystem::State::EXTENDED_FAST);
                else if (stickSlowButton)
                    stick.setState(StickSystem::State::EXTENDED_SLOW);
                else
                    stick.setState(StickSystem::State::RETRACTED);

                stick.moveStick();

                wings.setWingRaised(tube.getTubeRaised() == wingExtendButton);
                // if (!tube.getTubeRaised())
                //     wings.setWingRaised(!wingExtendButton);
                // else
                //     wings.setWingRaised(wingExtendButton);

                // if (tubeExtendButton && !prevTubeInput)
                //     tube.setTubeRaised(!tube.getTubeRaised());
                // prevTubeInput = tubeExtendButton;
                //wings.setWingRaised(wingExtendButton);
                if (tubeExtendButton)
                    tube.setTubeRaised(true);
                else
                    tube.setTubeRaised(false);



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
        static constexpr float DEAD_WHEEL_RADIUS = 1;
        static constexpr float MAX_ROBOT_SPEED_TO_SWITCH_DIRECTION = 0.5f;

        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(-1.8, 0);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0, -2.2);

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {10, -9, 8, -7, 6});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-1, 2, -3, 4, -5});

        SmartMotorGroup stickMotorsRight = SmartMotorGroup("StickMotorsRight", {-20});
        SmartMotorGroup stickMotorsLeft = SmartMotorGroup("StickMotorsLeft", {19});

        SmartMotorGroup topRoller = SmartMotorGroup("TopIntakeRoller", {16});
        SmartMotorGroup sideRollers = SmartMotorGroup("SideRollers", {-18, 17});

        ADIPneumatic tubePnematics = ADIPneumatic("TubePneumatics", 'C', false);
        ADIPneumatic wingPnematics = ADIPneumatic("WingPneumatics", 'F', false);
        ADIPneumaticGroup intakePnematics = ADIPneumaticGroup("IntakePneumatics", {'D', 'E'}, false);
        ADIPneumaticGroup ptoPnematics = ADIPneumaticGroup("PTOPneumatics", {'A', 'B'}, false);

        ADIDigitalInput stickHomeSensor = ADIDigitalInput("StickSensor", 'H');

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 12);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", -11);
        InertialSensor imu = InertialSensor("IMU", 13);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakePnematics, sideRollers, topRoller);
        StickSystem stick = StickSystem(
            ptoPnematics,
            stickMotorsLeft,
            stickMotorsRight,
            leftMotors,
            rightMotors,
            stickHomeSensor);
        TubeSystem tube = TubeSystem(tubePnematics);
        WingSystem wings = WingSystem(wingPnematics);

        std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
            verticalSensor,
            horizontalSensor,
            DEAD_WHEEL_RADIUS);

        // Displays
        // std::shared_ptr<DevilBotsDisplay> devilBotsDisplay = std::make_shared<DevilBotsDisplay>();
        // AutoPickerDisplay autoPickerDisplay = AutoPickerDisplay(
        //     "PJ Robot",
        //     {
        //         AutoPickerDisplay::Routine{.id = 0, .displayName = "Match Auto", .requiresAllianceColor = false},
        //         AutoPickerDisplay::Routine{.id = 1, .displayName = "Skills Auto", .requiresAllianceColor = false}
        //     });
        std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();
    };
}
