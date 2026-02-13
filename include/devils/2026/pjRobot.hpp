#pragma once

#include "../devils.h"
#include "./subsystems/intakeSystem.hpp"
#include "./autonomous/pjSkillsAuto.hpp"

namespace devils
{
    struct PJRobot : Robot
    {
        PJRobot()
        {
            imu.calibrate();

            odometry->useIMU(&imu);
            odometry->setSensorOffsets(VERTICAL_SENSOR_OFFSET, HORIZONTAL_SENSOR_OFFSET);
            odometry->start();
            
            toastDisplay->start();
            // devilBotsDisplay->start();
            
            // Logger::debug("This is a debug message");
            // Logger::info("This is an info message");
            // Logger::warn("This is a warning message");
            // Logger::error("This is an error message");
        }

        void autonomous() override
        {
            imu.waitUntilDoneCalibrated();
            
            auto autoBuilder = AutoBuilder(chassis, *odometry.get());
            autoBuilder.rotateTo(90, {.endingVelocity = 0.0f})->startSync();
            autoBuilder.rotateTo(180, {.endingVelocity = 0.0f})->startSync();
            // autoBuilder.rotateTo(0)->startSync();
            // autoBuilder.rotateTo(180)->startSync();
            
            // PJSkillsAuto::run(chassis, *odometry.get(), intake);
        }

        void opcontrol() override
        {
            // Drop Alignment Jig
            // intake.runIntake(-1.0f);
            // pros::delay(100);
            // intake.runIntake(0.0f);

            // Stop autonomous
            // AutoStep::stopAll();

            // Loop
            while (true)
            {
                // Take Controller Inputs
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                const bool exitCyclerButton = mainController.r1; // high goal from basket
                const bool midOuttakeButton = mainController.r2; // mid goal from ground
                const bool intakeExtendButton = mainController.y; // intake extend/retract
                const bool hoodExtendButton = mainController.l1; // hood extend/retract
                const bool exitCyclerMidButton = mainController.l2; // mid goal from cycler
                const bool rakePneumaticsButton = mainController.b; // toggle rake pneumatics

                // Combine Left and Right X Joystick Inputs
                const float combinedX = JoystickCurve::combine(leftX, rightX);

                // Run Cyclers
                if (exitCyclerButton)
                    intake.setIntakeMode(SideGoal); // Score Top
                else if (midOuttakeButton)
                    intake.setIntakeMode(MidBottom); // Score Bottom
                else if (exitCyclerMidButton)
                    intake.setIntakeMode(MidTop); // Score Mid
                else
                    intake.setIntakeMode(Cycler); // Intake to cycler

                bool isScoring = exitCyclerButton || midOuttakeButton || exitCyclerMidButton;

                intake.runIntake(isScoring ? 0.8f : rightY);
                intake.setArmsExtended(intakeExtendButton);
                intake.setHoodExtended(hoodExtendButton);

                rakePneumatics.setExtended(rakePneumaticsButton);

                // Drive normally
                chassis.move(leftY, combinedX * 0.5f, 0.0f);
                
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
        Vector2 VERTICAL_SENSOR_OFFSET = Vector2(0, 0);
        Vector2 HORIZONTAL_SENSOR_OFFSET = Vector2(0, 2.34);

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-11, 12, -13, 14, -15});

        SmartMotorGroup frontBottomIntakeMotors = SmartMotorGroup("FrontBottomIntake", {-1});
        SmartMotorGroup frontTopIntakeMotors = SmartMotorGroup("FrontTopIntake", {-10});
        SmartMotorGroup frontIntakeRollers = SmartMotorGroup("FrontIntakeRollers", {2, -3});
        SmartMotorGroup backIntakeMotors = SmartMotorGroup("BackIntakeMotors", {-5});
        SmartMotorGroup cyclerMotors = SmartMotorGroup("CyclerMotors", {-6});

        ADIPneumatic intakePneumaticsLeft = ADIPneumatic("IntakePneumatics", 'A');
        ADIPneumatic intakePneumaticsRight = ADIPneumatic("IntakePneumatics", 'B');
        ADIPneumatic rakePneumatics = ADIPneumatic("RakePneumatics", 'E');
        ADIPneumatic hoodPneumatics = ADIPneumatic("HoodPneumatics", 'H', true);

        OpticalSensor colorSensor = OpticalSensor("InventoryColorSensor", 7);
        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 9);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 8);
        InertialSensor imu = InertialSensor("IMU", 4);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(
            frontTopIntakeMotors,
            frontBottomIntakeMotors,
            frontIntakeRollers,
            backIntakeMotors,
            cyclerMotors,
            colorSensor,
            intakePneumaticsLeft,
            intakePneumaticsRight,
            hoodPneumatics);

        std::shared_ptr<PerpendicularSensorOdometry> odometry = std::make_shared<PerpendicularSensorOdometry>(
            verticalSensor,
            horizontalSensor,
            DEAD_WHEEL_RADIUS);
        
        // Displays
        // std::shared_ptr<DevilBotsDisplay> devilBotsDisplay = std::make_shared<DevilBotsDisplay>();
        std::shared_ptr<ToastDisplay> toastDisplay = std::make_shared<ToastDisplay>();

        // Renderer
        // RobotAutoOptions autoOptions = RobotAutoOptions();
        // std::vector<Routine> routines = {
        //     {0, "Default", false},
        // };
        // OptionsRenderer optionsRenderer = OptionsRenderer("PepperJack", routines, &autoOptions);
    };
}
