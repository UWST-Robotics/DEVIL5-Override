#pragma once

#include "devils/devils.h"

namespace devils
{
    class PJSkillsAuto
    {
        void run(ChassisBase &chassis,
            OdomSource &odometry)
        {
            // PID Params
            PIDController::Options rotatePID = {0.7, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.15, // minSpeed
                0.6,  // maxSpeed
                0.1,  // goalDist
            };
            TrajectoryConstraints slowConstraints = {24, 48};
            TrajectoryConstraints fastConstraints = {64, 92};

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-60, 0, 180)->start();
            
            // Drive to Trajectorys
            pjRoutine.driveToTrajectory(-68, 0, 180, false, 0, 1)->startSync();
            pjRoutine.pause(100)->startSync();

        }
    };
}