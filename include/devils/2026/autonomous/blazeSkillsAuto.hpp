#pragma once

#include "devils/devils.h"
#include "../subsystems/intakeSystem.hpp"
#include "../autonomous/autoIntakeStep.hpp"

namespace devils
{
    class BlazeSkillsAuto
    {
    public:
        static void run(ChassisBase& chassis,
                        OdomSource& odometry,
                        IntakeSystem& intake)
        {
            // PID Params
            const PIDController::Options rotatePID = {0.7, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.15, // minSpeed
                0.4, // maxSpeed
                0.1, // goalDist
            };
            const TrajectoryConstraints slowConstraints = {12, 48};

            // Initialize
            auto blazeRoutine = AutoBuilder(chassis, odometry);
            blazeRoutine.setPose(-48, 19, 270);
            intake.setHoodAutomatic(true);
            intake.setIntakeMode(IntakeMode::Cycler);

            auto intakeStep = std::make_shared<AutoIntakeStep>(intake, INTAKE_SPEED);
            intakeStep->start();

            // 1 (Pickup Preload)
            intake.setArmsExtended(true);
            blazeRoutine.driveToTrajectory(-48, 24, 270, {.isReversed = true, .strength = 0})->startSync();
            pauseForIntake(intakeStep, 1000);

            // 2 (SW Loader)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(-48, 52, 270, {.isReversed = true})->startSync();
            blazeRoutine.rotateTo(180)->startSync();
            intake.setArmsExtended(true);
            blazeRoutine.driveToTrajectory(-63, 50, 180)->startSync();
            pauseForIntake(intakeStep, 3000);

            // 3 (Score in Bottom Goal)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(-48, 51, 180, {.isReversed = true})->startSync();
            blazeRoutine.rotateTo(0)->startSync();
            blazeRoutine.driveToTrajectory(-30, 51, 0)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intakeStep);
            intake.setIntakeMode(IntakeMode::Cycler);

            // 4 (Ball underneath Bottom Goal)
            blazeRoutine.driveToTrajectory(-40, 44, 50, {.isReversed = true})->startSync();
            blazeRoutine.driveToTrajectory(-20, 64, 0, {.strength = 12, .finalVelocity = 12})->startSync();
            blazeRoutine.driveToTrajectory(-8, 56, -70, {.constraints = slowConstraints})->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(intakeStep);

            // 5 (SE Loader)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(-18, 62, 0, {.isReversed = true})->startSync();
            blazeRoutine.driveToTrajectory(20, 62, 0, {.finalVelocity = 12})->startSync();
            blazeRoutine.driveToTrajectory(60, 52, 0, {.strength = 16})->startSync();
            // blazeRoutine.pause(9999999)->startSync(); // Debug Pause
            intake.setArmsExtended(true);
            pauseForIntake(intakeStep, 1000);

            // 6 (Score in Bottom Goal)
            intake.setArmsExtended(false);
            blazeRoutine.driveToTrajectory(45, 52, 0, {.isReversed = true})->startSync();
            blazeRoutine.rotateTo(180)->startSync();
            blazeRoutine.driveToTrajectory(30, 52, 180)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intakeStep);
            intake.setIntakeMode(IntakeMode::Cycler);

            // TODO: Finish the rest of the path:
            // 7 (E Park Zone)
            // 8 (N Ball by center goals)
            // 9 (Score in center goal)
            // 10 (Park)
        }

    private:
        static void pauseForIntake(
            std::shared_ptr<AutoIntakeStep>& intake,
            uint32_t duration = 500)
        {
            // std::make_shared<AutoPauseStep>(duration)->startSync();
        }

        static void pauseForOuttake(
            const std::shared_ptr<AutoIntakeStep>& intake,
            const int32_t outtakeDuration = 5000,
            const float outtakeSpeed = 0.8f)
        {
            intake->setTargetSpeed(outtakeSpeed);
            // std::make_shared<AutoPauseStep>(outtakeDuration)->startSync();
            intake->setTargetSpeed(INTAKE_SPEED);
        }

        static constexpr double INTAKE_SPEED = 1.0;
        static constexpr double OUTTAKE_SLOW_SPEED = 0.6; // <-- Slower, avoids ball from overshooting the goal
        static constexpr double OUTTAKE_FAST_SPEED = 1.0; // <-- Faster, uses existing balls as a backstop
    };
}
