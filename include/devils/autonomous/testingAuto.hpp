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
                .maxVelocity = 10.0f
            };

            AutoBuilder autoBuilder = AutoBuilder(chassis, odom);
            autoBuilder.jumpTo({0, 0, Units::degToRad(0)});
            autoBuilder.driveToHolonomic({48,0, Units::degToRad(0)})->startSync();
            autoBuilder.driveToLinear({24,24,Units::degToRad(180)})->startSync();
            autoBuilder.driveTo({-24, 24, Units::degToRad(90)})->startSync();
            autoBuilder.driveTo({24, 24, Units::degToRad(-90)})->startSync();
            autoBuilder.driveTo({24, -24, Units::degToRad(0)})->startSync();
            autoBuilder.driveTo({-24, -24, Units::degToRad(90)})->startSync();
            autoBuilder.driveTo({-0, 0, Units::degToRad(0)})->startSync();
            //autoBuilder.rotate(180)->startSync();
        }
    };
}
