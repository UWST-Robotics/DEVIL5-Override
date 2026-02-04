#pragma once

#include "devils/devils.h"
#include "../subsystems/intakeSystem.hpp"
#include "../autonomous/autoIntakeStep.hpp"

namespace devils
{
    class PJSkillsAuto
    {
    public:
        static void run(ChassisBase& chassis,
                        OdomSource& odometry,
                        IntakeSystem& intake)
        {
            // PID Params
            PIDController::Options rotatePID = {0.7, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.15, // minSpeed
                0.4, // maxSpeed
                0.1, // goalDist
            };
            TrajectoryConstraints slowConstraints = {12, 48};
            TrajectoryConstraints fastConstraints = {72, 102};

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-45, 0, 180);
            intake.setHoodAutomatic(true);
            intake.setIntakeMode(IntakeMode::Cycler);

            auto intakeStep = std::make_shared<AutoIntakeStep>(intake, INTAKE_SPEED);
            intakeStep->start();

            // 1 (W Park Zone)
            pjRoutine.driveToTrajectory(-37, 0, 180, {.isReversed = true, .constraints = slowConstraints})->startSync();
            pauseForIntake(1000);

            // 2 (N Side Ball)
            pjRoutine.rotateTo(280)->startSync();
            intake.setArmsExtended(true);
            pjRoutine.driveToTrajectory(-16, -36, 340, {.strength = 18, .finalVelocity = 10})->startSync();
            pjRoutine.driveToTrajectory(-6, -38, 0, {.constraints = slowConstraints})->startSync();
            pauseForIntake();

            // 3 (NW Side Balls)
            intake.setArmsExtended(false);
            pjRoutine.driveToTrajectory(-24, -30, 340, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(200)->startSync();
            pjRoutine.driveToTrajectory(-45, -60, 270, {.strength = 16, .finalVelocity = 12})->startSync();
            pjRoutine.driveToTrajectory(-45, -66, 270, {.constraints = slowConstraints})->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(2500);
            intake.setArmsExtended(false);

            // 4 (Score in Center Goal)
            pjRoutine.driveToTrajectory(-45, -54, 270, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(56)->startSync();
            pjRoutine.driveToTrajectory(-8, -12, 45, {.strength = 12})->startSync();
            intake.setIntakeMode(IntakeMode::MidTop);
            pauseForOuttake(intakeStep, 3000);
            intake.setIntakeMode(IntakeMode::Cycler);

            // 5 (NW Loader)
            pjRoutine.driveToTrajectory(-40, -48, 67, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(180)->startSync();
            pjRoutine.driveToTrajectory(-60, -47, 180)->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(3000);
            intake.setArmsExtended(false);

            // 6 (Score in Top Goal)
            pjRoutine.driveToTrajectory(-40, -48, 180, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(0)->startSync();
            pjRoutine.driveToTrajectory(-26, -48, 0)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intakeStep);
            intake.setIntakeMode(IntakeMode::Cycler);

            // 7 (NE Side Balls)
            pjRoutine.driveToTrajectory(-46, -46, 300, {.isReversed = true, .strength = 6})->startSync();
            pjRoutine.driveToTrajectory(-28, -60, 350, {.strength = 12, .finalVelocity = 16})->startSync();
            pjRoutine.driveToTrajectory(28, -60, 10, {.strength = 12, .finalVelocity = 16})->startSync();
            pjRoutine.driveToTrajectory(52, -48, 45, {.strength = 12})->startSync();
            pjRoutine.rotateTo(270)->startSync();
            intake.runIntake(INTAKE_SPEED);
            pjRoutine.driveToTrajectory(52, -62, 270, {.constraints = slowConstraints})->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(2500);
            intake.setArmsExtended(false);

            // 8 (NE Loader)
            pjRoutine.driveToTrajectory(52, -46, 270, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(0)->startSync();
            pjRoutine.driveToTrajectory(68, -46, 0)->startSync();
            intake.setArmsExtended(true);
            pauseForIntake(3000);
            intake.setArmsExtended(false);

            // 9 (Score in Top Goal)
            pjRoutine.driveToTrajectory(54, -46, 0, {.isReversed = true})->startSync();
            pjRoutine.rotateTo(180)->startSync();
            pjRoutine.driveToTrajectory(32, -46, 180)->startSync();
            intake.setIntakeMode(IntakeMode::SideGoal);
            pauseForOuttake(intakeStep);
            intake.setIntakeMode(IntakeMode::Cycler);

            // 10 (Park Zone)
            pjRoutine.driveToTrajectory(60, -54, 140, {.isReversed = true})->startSync();
            pjRoutine.driveToTrajectory(0, -24, 180, {.strength = 16, .finalVelocity = 16})->startSync();
            pjRoutine.driveToTrajectory(-38, -32, 180, {.strength = 12, .finalVelocity = 16})->startSync();
            pjRoutine.driveToTrajectory(-60, -18, 90, {
                                            .strength = 16, .finalVelocity = 18, .constraints = fastConstraints
                                        })->startSync();
            pjRoutine.driveToTrajectory(-60, 12, 90, {.finalVelocity = 0, .constraints = fastConstraints})->startSync();
        }

    private:
        static void pauseForIntake(uint32_t duration = 500)
        {
            std::make_shared<AutoPauseStep>(duration)->startSync();
        }

        static void pauseForOuttake(
            std::shared_ptr<AutoIntakeStep>& intake,
            uint32_t outtakeDuration = 5000,
            float outtakeSpeed = 0.8f)
        {
            intake->setTargetSpeed(outtakeSpeed);
            std::make_shared<AutoPauseStep>(outtakeDuration)->startSync();
            intake->setTargetSpeed(INTAKE_SPEED);
        }

        static constexpr float INTAKE_SPEED = 1.0;
    };
}
