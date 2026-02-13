#pragma once

#include "../devils.h"

namespace devils
{
    struct DummyRobot : Robot
    {
        DummyRobot()
        {
            chassis->start();
        }
        
        void opcontrol() override
        {
            OdomSource& odom = *chassis;
            ChassisBase& chassisBase = *chassis;
            
            auto autoBuilder = AutoBuilder(chassisBase, odom);
            
            AutoRotateToStep::Options::defaultOptions.useMinimumDistance = false;
            const auto testStep = autoBuilder.rotate(360)->start();
            
            while (testStep->getState() != STOPPED)
            {
                const auto currentRotation = chassis->getPose().rotation;
                Logger::info(std::to_string(Units::radToDeg(currentRotation)));
                pros::delay(20);
            }
        }
        
        std::shared_ptr<DummyChassis> chassis = std::make_shared<DummyChassis>();
    };
}
