#pragma once

#include "../utils/asyncTask.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/gyroBase.h"
#include "devils/devilLib/chassis/swerveChassis.hpp"
#include "poseVelocityCalculator.hpp"

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
            // Code base on https://kauffman.tools/robotics/engineering/math/2025/09/16/swervedrive-forwards-kinematics.html
            float FLDelta = chassis.getWheelAngleInfo(0) - lastFLDist;
            Vector2 FLVect = Vector2(FLDelta * std::cos(chassis.getWheelAngleInfo(0)), FLDelta * std::sin(chassis.getWheelAngleInfo(0)));

            float FRDelta = chassis.getWheelAngleInfo(1) - lastFRDist;
            Vector2 FRVect = Vector2(FRDelta * std::cos(chassis.getWheelAngleInfo(1)), FRDelta * std::sin(chassis.getWheelAngleInfo(1)));

            float BLDelta = chassis.getWheelAngleInfo(2) - lastBLDist;
            Vector2 BLVect = Vector2(BLDelta * std::cos(chassis.getWheelAngleInfo(2)), BLDelta * std::sin(chassis.getWheelAngleInfo(2)));

            float BRDelta = chassis.getWheelAngleInfo(3) - lastBRDist;
            Vector2 BRVect = Vector2(BRDelta * std::cos(chassis.getWheelAngleInfo(3)), BRDelta * std::sin(chassis.getWheelAngleInfo(3)));

            // Calculate Delta Distance
            const Vector2 deltaDistance = (FLVect + FRVect + BLVect + BRVect) * 0.25;

            // Update IMU
            // Also calculate the change in rotation
            float deltaRotation = 0;
            if (imu != nullptr && imu->getIsReady())
            {
                const auto headingResult = imu->getHeading();
                if (headingResult.isSuccess())
                {
                    deltaRotation = headingResult.value - lastRotation;
                    lastRotation = headingResult.value;
                    currentPose.rotation = lastRotation;
                }
            }

            // Calculate trigonometric values
            const float rotation = currentPose.rotation;
            const float sin = std::sin(rotation);
            const float cos = std::cos(rotation);

            const float deltaX = deltaDistance.y * cos + deltaDistance.x * sin;
            const float deltaY = deltaDistance.y * sin - deltaDistance.x * cos;

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
