#pragma once

#include "devils/devilLib/devils.h"

namespace devils
{
    class TestingAuto
    {
    public:
        static void run(SwerveChassis& chassis, OdomSource& odom)
        {
            TrajectoryConstraints::defaultConstraints = TrajectoryConstraints{
                .maxVelocity = 25.0f
            };

            AutoBuilder autoBuilder = AutoBuilder(chassis, odom);
            autoBuilder.jumpTo({0, 0, Units::degToRad(0)});
            autoBuilder.driveToHolonomic({24,0,Units::degToRad(0)})->startSync();
            autoBuilder.driveToHolonomic({-24, 0, Units::degToRad(25)})->startSync();
            autoBuilder.driveToHolonomic({24,0,Units::degToRad(0)})->startSync();
            autoBuilder.driveToHolonomic({-24, 0, Units::degToRad(25)})->startSync();
            autoBuilder.driveToHolonomic({24,0,Units::degToRad(0)})->startSync();
            autoBuilder.driveToHolonomic({-24, 0, Units::degToRad(25)})->startSync();
            autoBuilder.driveToHolonomic({0, 0, Units::degToRad(0)})->startSync();
            //autoBuilder.rotate(180)->startSync();
        }
    };
}
