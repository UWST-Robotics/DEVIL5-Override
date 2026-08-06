#pragma once

#include "devils/devilLib/devils.h"
#include "../subsystems/liftSystem.hpp"
#include "../subsystems/clawSystem.hpp"

namespace devils
{
    class TestingAuto
    {
    public:
        static void run(SwerveChassis& chassis, OdomSource& odom, LiftSystem& lift, ClawSystem& claw)
        {
            TrajectoryConstraints::defaultConstraints = TrajectoryConstraints{
                .maxVelocity = 3.0f
            };

            AutoBuilder autoBuilder = AutoBuilder(chassis, odom);
            autoBuilder.jumpTo({0, 0, Units::degToRad(0)});
            autoBuilder.driveToHolonomic({0,24, Units::degToRad(90)}, Units::degToRad(0), Units::degToRad(180))->startSync();
            //autoBuilder.driveToLinear({24,24,Units::degToRad(180)})->startSync();
            //autoBuilder.driveTo({-24, 24, Units::degToRad(0)})->startSync();
            //autoBuilder.driveTo({24, 24, Units::degToRad(0)})->startSync();
            //autoBuilder.driveTo({24, -24, Units::degToRad(0)})->startSync();
            //autoBuilder.driveTo({-24, -24, Units::degToRad(0)})->startSync();
            //autoBuilder.rotate(180);
        }
    };
}
