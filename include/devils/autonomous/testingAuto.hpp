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
                .maxVelocity = 20.0f
            };
            
            auto autoBuilder = AutoBuilder(chassis, odom);

            autoBuilder.jumpTo({0, 0, Units::degToRad(180)});
            autoBuilder.driveTo({-24, 24, Units::degToRad(0)})->startSync();
            //autoBuilder.rotate(180);
        }
    };
}
