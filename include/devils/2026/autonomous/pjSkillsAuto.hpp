#pragma once

#include "devils/devils.h"
#include "../subsystems/intakeSystem.hpp"

namespace devils
{
    class PJSkillsAuto
    {
    public:
        static void run(ChassisBase &chassis,
                        OdomSource &odometry,
                        IntakeSystem &intake)
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
            pjRoutine.setPose(-48, 0, 180)->start();
            intake.defaultIntake(1.0f);

            // 1 (Park Zone)
            pjRoutine.driveToTrajectory(-40, 0, 180, true, 0, 1, slowConstraints)->startSync();
            pauseForIntake();

            // 2 (N Side Ball)
            pjRoutine.rotateTo(280)->startSync();
            pjRoutine.driveToTrajectory(-16, -36, 340)->startSync();
            pjRoutine.driveToTrajectory(-12, -38, 340, false, 0, 1, slowConstraints)->startSync();
            pauseForIntake();

            // 3 (NW Side Balls)
            pjRoutine.driveToTrajectory(-16, -36, 340, true)->startSync();
            pjRoutine.rotateTo(170)->startSync();
            pjRoutine.driveToTrajectory(-48, -64, 270)->startSync();
            pauseForIntake();

            // 4 (Score in Center Goal)
            pjRoutine.driveToTrajectory(-48, -56, 270, true)->startSync();
            pjRoutine.rotateTo(56);
            pjRoutine.driveToTrajectory(-16, -16, 45)->startSync();
            intake.outtakeMid();
            pauseForOuttake();
            intake.stopOuttake();

            // 5 (NW Loader)
            pjRoutine.driveToTrajectory(-40, -52, 67, true)->startSync();
            pjRoutine.rotateTo(180)->startSync();
            pjRoutine.driveToTrajectory(-60, -52, 180)->startSync();
            intake.intakeExtend();
            intake.defaultIntake(1.0f);
            pauseForIntake();
            intake.intakeRetract();

            // 6 (Score in Top Goal)
            pjRoutine.driveToTrajectory(-40, -52, 180, true)->startSync();
            pjRoutine.rotateTo(0)->startSync();
            pjRoutine.driveToTrajectory(-30, -52, 0)->startSync();
            intake.outtakeTop();
            pauseForOuttake();
            intake.stopOuttake();

            // 7 (NE Side Balls)
            pjRoutine.driveToTrajectory(-40, -52, 0, true)->startSync();
            pjRoutine.driveToTrajectory(-28, -62, 350, false, 12)->startSync();
            pjRoutine.driveToTrajectory(22, -62, 10, false, 12)->startSync();
            pjRoutine.driveToTrajectory(48, -48, 45)->startSync();
            pjRoutine.rotateTo(270)->startSync();
            pjRoutine.driveToTrajectory(48, -62, 270)->startSync();
            pauseForIntake();

            // 8 (NE Loader)
            pjRoutine.driveToTrajectory(48, -48, 270, true)->startSync();
            pjRoutine.rotateTo(0)->startSync();
            pjRoutine.driveToTrajectory(62, -48, 0)->startSync();
            intake.intakeExtend();
            intake.defaultIntake(1.0f);
            pauseForIntake();
            intake.intakeRetract();

            // 9 (Score in Top Goal)
            pjRoutine.driveToTrajectory(48, -48, 0, true)->startSync();
            pjRoutine.rotateTo(180)->startSync();
            pjRoutine.driveToTrajectory(32, -48, 180)->startSync();
            intake.outtakeTop();
            pauseForOuttake();
            intake.stopOuttake();

            // 10 (Park Zone)
            pjRoutine.driveToTrajectory(48, -48, 180, true)->startSync();
            pjRoutine.driveToTrajectory(0, -24, 180, false, 12)->startSync();
            pjRoutine.driveToTrajectory(-48, -36, 180, false, 12)->startSync();
            pjRoutine.driveToTrajectory(-64, -18, 90, false, 12)->startSync();
            pjRoutine.driveToTrajectory(-64, 6, 90, false, 0, 1, fastConstraints)->startSync();
        }

    private:
        static void pauseForIntake(uint32_t duration = 500)
        {
            std::make_shared<AutoPauseStep>(duration)->startSync();
        }

        static void pauseForOuttake(uint32_t duration = 3000)
        {
            std::make_shared<AutoPauseStep>(duration)->startSync();
        }
    };
}