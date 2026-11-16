#pragma once

#include "../utils/asyncTask.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/gyroBase.h"
#include "devils/devilLib/chassis/swerveChassis.hpp"
#include "devils/devilLib/utils/logger.hpp"
#include "poseVelocityCalculator.hpp"
#include <string>

namespace devils
{
    class SwerveOdometry2 :
        public OdomSource,
        public AsyncTask,
        public PoseVelocityCalculator
    {
    public:
        /**
         * Creates an odometry system using a swerve chassis.
         */
        SwerveOdometry2(
            SwerveChassis& chassis,
            SwerveConstants& swerveConstants)
            : chassis(chassis),
              swerveConstants(swerveConstants)
        {
    
        }

        /**
         * Gets the current pose of the robot.
         */
        Pose getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(const Pose pose) override
        {
            currentPose = pose;
            lastRotation = pose.rotation;

            if (imu != nullptr)
                imu->setHeading(pose.rotation);
        }

        /**
         * Sets the IMU to use for odometry.
         * @param newIMU The new IMU to use for odometry.
         */
        void useIMU(IGyro* newIMU)
        {
            this->imu = newIMU;
        }

        PoseVelocity getVelocity() override
        {
            return PoseVelocityCalculator::getVelocity();
        }

    protected:
        /**
         * Updates the odometry from swerve modules and the IMU
         */
        void onUpdate() override
        {
            // Code based on https://kauffman.tools/robotics/engineering/math/2025/09/16/swervedrive-forwards-kinematics.html
            float FLDelta = chassis.getWheelDistanceInfo(0) - lastFLDist;
            lastFLDist = chassis.getWheelDistanceInfo(0);
            Vector2 FLVect = Vector2(FLDelta * std::cos(chassis.getWheelAngleInfo(0)), FLDelta * -std::sin(chassis.getWheelAngleInfo(0)));
            
            float FRDelta = chassis.getWheelDistanceInfo(1) - lastFRDist;
            lastFRDist = chassis.getWheelDistanceInfo(1);
            Vector2 FRVect = Vector2(FRDelta * std::cos(chassis.getWheelAngleInfo(1)), FRDelta * -std::sin(chassis.getWheelAngleInfo(1)));

            float BLDelta = chassis.getWheelDistanceInfo(2) - lastBLDist;
            lastBLDist = chassis.getWheelDistanceInfo(2);
            Vector2 BLVect = Vector2(BLDelta * std::cos(chassis.getWheelAngleInfo(2)), BLDelta * -std::sin(chassis.getWheelAngleInfo(2)));

            float BRDelta = chassis.getWheelDistanceInfo(3) - lastBRDist;
            lastBRDist = chassis.getWheelDistanceInfo(3);
            Vector2 BRVect = Vector2(BRDelta * std::cos(chassis.getWheelAngleInfo(3)), BRDelta * -std::sin(chassis.getWheelAngleInfo(3)));

            // Calculate Delta Distance
            const Vector2 deltaDistance = (FLVect + FRVect + BLVect + BRVect) * 0.25;
            Logger::info("Delta distance local: " + deltaDistance.toString());

            // Update IMU
            if (imu != nullptr && imu->getIsReady())
            {
                const auto headingResult = imu->getHeading();
                if (headingResult.isSuccess())
                {
                    lastRotation = headingResult.value;
                    currentPose.rotation = lastRotation;
                }
            }

            // Calculate trigonometric values
            const float rotation = currentPose.rotation;

            const float deltaX = deltaDistance.rotateInAustinCoords(rotation).x;
            const float deltaY = deltaDistance.rotateInAustinCoords(rotation).y;

            Logger::info("Delta distance global: " + std::to_string(deltaX) + ", " + std::to_string(deltaY));

            // Update Pose
            currentPose.x += deltaX;
            currentPose.y += deltaY;

            // Update Velocity
            updateVelocity(currentPose);
        }

    private:
        SwerveChassis chassis;
        SwerveConstants swerveConstants;

        IGyro* imu = nullptr;

        Pose currentPose = Pose();

        float lastFLDist = 0;
        float lastFRDist = 0;
        float lastBLDist = 0;
        float lastBRDist = 0;

        float lastRotation = 0;
    };
}
