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
                0.4,  // maxSpeed
                0.1,  // goalDist
            };
            TrajectoryConstraints slowConstraints = {12, 48};
            TrajectoryConstraints fastConstraints = {64, 92};

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-48, 0, 180);
            intake.setHoodAutomatic(true);
            intake.setIntakeMode(IntakeMode::Cycler);
            intake.runIntake(INTAKE_SPEED);

            // 1 (W Park Zone)
            pjRoutine.driveToTrajectory(-40, 0, 180, {.isReversed = true, .constraints = slowConstraints})->startSync();
            pauseForIntake(intake, 1000);

            // 2 (N Side Ball)
            pjRoutine.rotateTo(280)->startSync();
            intake.setArmsExtended(true);
            pjRoutine.driveToTrajectory(-16, -36, 340, {.strength = 18, .finalVelocity = 10})->startSync();
            pjRoutine.driveToTrajectory(-6, -38, 0, {.constraints = slowConstraints})->startSync();
            pauseForIntake(intake);

            // 3 (NW Side Balls)
            pjRoutine.driveToTrajectory(-26, -32, 340, {.isReversed = true})->startSync();
            intake.setArmsExtended(false);
            pjRoutine.rotateTo(200)->startSync();
            pjRoutine.driveToTrajectory(-45, -60, 270, {.strength = 16, .finalVelocity = 12})->startSync();
            pjRoutine.driveToTrajectory(-45, -66, 270, {.constraints = slowConstraints})->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intake, 2500);
            intake.setArmsExtended(false);

            // 4 (Score in Center Goal)
            pjRoutine.driveToTrajectory(-45, -54, 270, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(56)->startSync();
            pjRoutine.driveToTrajectory(-6, -14, 45, {.strength = 12})->startSync();
            intake.setIntakeMode(IntakeMode::MidTop); // << MidBottom actually
            pauseForOuttake(intake);
            intake.setIntakeMode(IntakeMode::Cycler);

            // 5 (NW Loader)
            pjRoutine.driveToTrajectory(-40, -48, 67, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(180)->startSync();
            pjRoutine.driveToTrajectory(-60, -48, 180)->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intake, 3000);
            intake.setArmsExtended(false);

            // 6 (Score in Top Goal)
            pjRoutine.driveToTrajectory(-40, -49, 180, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(0)->startSync();
            pjRoutine.driveToTrajectory(-24, -50, 0)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intake);
            intake.setIntakeMode(IntakeMode::Cycler);

            // 7 (NE Side Balls)
            pjRoutine.driveToTrajectory(-46, -46, 300, {.isReversed = true})->startSync();
            pjRoutine.driveToTrajectory(-28, -62, 350, {.strength = 12, .finalVelocity = 12, .constraints = slowConstraints})->startSync();
            pjRoutine.driveToTrajectory(28, -62, 10, {.strength = 12, .finalVelocity = 12})->startSync();
            pjRoutine.driveToTrajectory(54, -48, 45, {.strength = 12})->startSync();
            pjRoutine.rotateTo(270)->startSync();
            intake.runIntake(INTAKE_SPEED);
            pjRoutine.driveToTrajectory(54, -62, 270, {.constraints = slowConstraints})->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intake);
            intake.setArmsExtended(false);

            // 8 (NE Loader)
            pjRoutine.driveToTrajectory(54, -48, 270, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(0)->startSync();
            pjRoutine.driveToTrajectory(68, -48, 0)->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intake, 3000);
            intake.setArmsExtended(false);

            // 9 (Score in Top Goal)
            pjRoutine.driveToTrajectory(54, -48, 0, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(180)->startSync();
            pjRoutine.driveToTrajectory(38, -48, 180)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intake);
            intake.setIntakeMode(IntakeMode::Cycler);
            pjRoutine.pause(999999999)->startSync();

            // 10 (Park Zone)
            pjRoutine.driveToTrajectory(48, -48, 180, {.isReversed = true})->startSync();
            pjRoutine.driveToTrajectory(0, -24, 180, {.strength = 12, .finalVelocity = 12})->startSync();
            pjRoutine.driveToTrajectory(-48, -36, 180, {.strength = 12, .finalVelocity = 12})->startSync();
            pjRoutine.driveToTrajectory(-64, -18, 90, {.strength = 12, .finalVelocity = 12})->startSync();
            pjRoutine.driveToTrajectory(-64, 6, 90, {.finalVelocity = 0, .constraints = fastConstraints})->startSync();
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