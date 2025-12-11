#pragma once

#include "devils/devils.h"
#include "../subsystems/intakeSystem.hpp"

namespace devils
{
    class BlazeSkillsAuto
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
                0.4,  // maxSpeed
                0.1,  // goalDist
            };
            TrajectoryConstraints slowConstraints = {12, 48};
            TrajectoryConstraints fastConstraints = {64, 92};

            // Initialize
            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);
            blazeRoutine.setPose(-48, 16, 270);
            intake.setHoodAutomatic(true);
            intake.setIntakeMode(IntakeMode::Cycler);
            intake.runIntake(INTAKE_SPEED);

            // 1 (Pickup Preload)
            intake.setArmsExtended(true);
            blazeRoutine.driveToTrajectory(-48, 14, 270, {.strength = 0, .constraints = slowConstraints})->startSync();
            pauseForIntake(intake, 1000);

            // 2 (SW Loader)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(-48, 52, 270, {.isReversed = true})->startSync();
            blazeRoutine.rotateTo(180)->startSync();
            blazeRoutine.driveToTrajectory(-62, 50, 180)->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intake, 3000);

            // 3 (Score in Bottom Goal)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(-48, 50, 180, {.isReversed = true})->startSync();
            blazeRoutine.rotateTo(0)->startSync();
            blazeRoutine.driveToTrajectory(-30, 50, 0)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intake);
            intake.setIntakeMode(IntakeMode::Cycler);

            // 4 (Ball underneath Bottom Goal)
            blazeRoutine.driveToTrajectory(-40, 44, 50, {.isReversed = true})->startSync();
            blazeRoutine.driveToTrajectory(-18, 62, 0, {.strength = 12, .finalVelocity = 12})->startSync();
            blazeRoutine.driveToTrajectory(-8, 58, -50, {.constraints = slowConstraints})->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intake, 1000);

            // 5 (SE Loader)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(-18, 62, 0, {.isReversed = true})->startSync();
            blazeRoutine.driveToTrajectory(20, 62, 0, {.finalVelocity = 12})->startSync();
            blazeRoutine.driveToTrajectory(60, 52, 0, {.strength = 16})->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intake, 1000);

            // 6 (Score in Bottom Goal)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(45, 52, 0, {.isReversed = true})->startSync();
            blazeRoutine.rotateTo(180)->startSync();
            blazeRoutine.driveToTrajectory(30, 52, 180)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intake);
            intake.setIntakeMode(IntakeMode::Cycler);

            // TODO: Finish the rest of the path:
            // 7 (E Park Zone)
            // 8 (N Ball by center goals)
            // 9 (Score in center goal)
            // 10 (Park)
        }

    private:
        static void pauseForIntake(
            IntakeSystem &intake,
            uint32_t duration = 500)
        {
            std::make_shared<AutoPauseStep>(duration)->startSync();
        }

        static void pauseForOuttake(
            IntakeSystem &intake,
            uint32_t slowDuration = 3000,
            uint32_t fastDuration = 1000)
        {
            intake.runIntake(OUTTAKE_SLOW_SPEED);
            std::make_shared<AutoPauseStep>(slowDuration)->startSync();
            intake.runIntake(OUTTAKE_FAST_SPEED);
            std::make_shared<AutoPauseStep>(fastDuration)->startSync();
            intake.runIntake(0.0f);
        }

        static constexpr double INTAKE_SPEED = 1.0;
        static constexpr double OUTTAKE_SLOW_SPEED = 0.6; // <-- Slower, avoids ball from overshooting the goal
        static constexpr double OUTTAKE_FAST_SPEED = 1.0; // <-- Faster, uses existing balls as a backstop
    };
}