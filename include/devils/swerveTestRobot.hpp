#pragma once

#include "./devilLib/devils.h"
#include "./autonomous/matchAuto.hpp"
#include "./autonomous/skillsAuto.hpp"
#include "./subsystems/intakeSystem.hpp"
#include "./subsystems/StickSystem.hpp"
#include "./subsystems/tubeSystem.hpp"
#include "./subsystems/wingSystem.hpp"
#include "./subsystems/swerveDisplay.hpp"

namespace devils
{
    struct SwerveTestRobot : Robot
    {
        // Constants
        
        // Hardware
        SmartMotorGroup frontLeftMotorA = SmartMotorGroup("FrontLeftMotorA", {1});
        SmartMotorGroup frontLeftMotorB = SmartMotorGroup("FrontLeftMotorB", {2});

        ADIAnalogInput frontLeftEncoder = ADIAnalogInput("FrontLeftEncoder", 'H', false);

        // Drivetrain
        SwerveModule frontLeftModule = SwerveModule(frontLeftMotorA, frontLeftMotorB, frontLeftEncoder, M_PI);

        // Display (don't know how to use this Austin)
        //std::shared_ptr<SwerveDisplay> swerveDisplay = std::make_shared<SwerveDisplay>();

        // Vexbridge (yikes)
        VEXBridge bridge = VEXBridge();
        // Dummy Chassis
        DummyChassis dummyChassis = DummyChassis();
        // Odom currently crashes the brain for some reason
        //VBOdom dummyOdomNT = VBOdom("DummyOdom", dummyChassis);
        VBValue<float> testval = VBValue("test", 12.0f);
        VBValue<float> vbAngle = VBValue("angle", pointingAngle);
        float pointingAngle = 0.0;

        // Ok acutal code now
        SwerveTestRobot()
        {

            //swerveDisplay->start();

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
            //SkillsAuto::run(chassis, *odometry.get(), stick, intake, tube, wings, true);
            //MatchAuto::run(chassis, *odometry.get(), stick, intake, tube, wings, true);
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

                // Drive the robot
                frontLeftModule.move(pointingAngle, leftY);
                pointingAngle += rightX * 0.3;
                //std::cout << "pointing angle: " << pointingAngle;
                //std:cout << "\n";

                vbAngle.set(pointingAngle);
                // Delay to prevent the CPU from being overloaded
                pros::delay(10);
            }
        }

        void disabled() override
        {
            // Stop the robot
            //swerve.stop();

            // Stop autonomous
            AutoStep::stopAll();
        }
    };
}
