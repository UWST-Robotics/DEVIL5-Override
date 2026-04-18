#pragma once

#include "devils/devils.h"
#include "../subsystems/StickSystem.hpp"
#include "../subsystems/intakeSystem.hpp"
#include "../subsystems/tubeSystem.hpp"
#include "../subsystems/wingSystem.hpp"
#include "stickAutoStep.hpp"
#include "autoIntakeStep.hpp"

namespace devils
{
    class TubeSystem;
    class IntakeSystem;

    class pjAutoDeploy
    {
    public:
        static void run(TankChassis& chassis,
                        OdomSource& odom,
                        StickSystem& stick,
                        IntakeSystem& intake,
                        TubeSystem& tube,
                        WingSystem& wings,
                        Controller& mainController
                        )
        {   
            const auto stickAutoStep = std::make_shared<StickAutoStep>(stick);
            stickAutoStep->start();

            const auto intakeAutoStep = std::make_shared<AutoIntakeStep>(intake, stick);
            intakeAutoStep->start();

            tube.setTubeRaised(true);
            opControl(mainController, chassis, 0.5);
            stick.setState(StickSystem::State::EXTEND_FOR_PJ_DEPLOY);
            opControlOnPosition(mainController, chassis, 1.5, stick);
            stick.setState(StickSystem::State::RETRACTED);
        }

        static void opControl(Controller& mainController,
        TankChassis& chassis, float delayTime)
        {
            Timer timer(delayTime);
            timer.start();
            while (timer.getIsRunning()) {
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                const float combinedX = Math::largestMagnitude({leftX, rightX});
                const auto driveDirection = 1.0f;
                chassis.move(leftY * driveDirection, combinedX * 0.75f, 0);
                pros::delay(20);
            }
        }

        static void opControlOnPosition(Controller& mainController, TankChassis& chassis, float delayTime, StickSystem stickSystem)
        {
            Timer timer(delayTime);
            timer.start();
            while (stickSystem.getIsBusy() && timer.getIsRunning()) {
                const float leftY = mainController.leftY;
                const float leftX = mainController.leftX;
                const float rightY = mainController.rightY;
                const float rightX = mainController.rightX * 0.5f;

                const float combinedX = Math::largestMagnitude({leftX, rightX});
                const auto driveDirection = 1.0f;
                chassis.move(leftY * driveDirection, combinedX * 0.75f, 0);
                pros::delay(20);
            }
        }
    };
}
