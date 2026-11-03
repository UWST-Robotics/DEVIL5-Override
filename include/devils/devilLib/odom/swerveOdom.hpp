#pragma once
#include "../geometry/pose.hpp"
#include "devils/devilLib/chassis/swerveChassis.hpp"
#include "devils/devilLib/geometry/vector2.hpp"
#include "odomSource.hpp"
#include "poseVelocityCalculator.hpp"
#include "pros/rtos.hpp"
#include <cmath>

#define M_PI 3.14159265358979323846

namespace devils
{
    /**
     * Represents an odometry system using a set of swerve modules
     */
    class SwerveOdom : public OdomSource, public PoseVelocityCalculator
    {
    public:
        /**
         * Creates a new odometry system using a set of four swerve modules.
         * Position is calculated using the velocities of all four modules over time.
         * @param chassis The swerve chassis to extract wheel information from
         */
        SwerveOdom(SwerveChassis chassis, SwerveConstants swerveConstants) : chassis(chassis), swerveConstants(swerveConstants)
        {
            lastUpdateTimestamp = pros::millis();
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
        }

        /**
         * Updates the odometry from the four swerve pods
         */
        void update()
        {
            // Get Delta Time
            lastUpdateTimestamp = pros::millis();

            // TODO: write all of this
            // Code base on https://kauffman.tools/robotics/engineering/math/2025/09/16/swervedrive-forwards-kinematics.html

            float FLDelta = chassis.getWheelAngleInfo(0) - lastFLDist;
            Vector2 FLVect = Vector2(FLDelta * Math.cos(chassis.getWheelAngleInfo(0)), FLDelta * Math.sin(chassis.getWheelAngleInfo(0)));

            float FRDelta = chassis.getWheelAngleInfo(1) - lastFRDist;
            Vector2 FRVect = Vector2(FRDelta * Math.cos(chassis.getWheelAngleInfo(1)), FRDelta * Math.sin(chassis.getWheelAngleInfo(1)));

            float BLDelta = chassis.getWheelAngleInfo(2) - lastBLDist;
            Vector2 BLVect = Vector2(BLDelta * Math.cos(chassis.getWheelAngleInfo(2)), BLDelta * Math.sin(chassis.getWheelAngleInfo(2)));

            float BRDelta = chassis.getWheelAngleInfo(3) - lastBRDist;
            Vector2 BRVect = Vector2(BRDelta * Math.cos(chassis.getWheelAngleInfo(3)), BRDelta * Math.sin(chassis.getWheelAngleInfo(3)));


            // Calculate Delta Distance
            const Vector2 deltaDistance = (FLVect + FRVect + BLVect + BRVect) * 0.25;
            const float deltaRotation = (
                (FLVect.dot(swerveConstants.flTurnVect) / FLVect.magnitude() +
                FRVect.dot(swerveConstants.frTurnVect) / FRVect.magnitude() +
                BLVect.dot(swerveConstants.blTurnVect) / BLVect.magnitude() +
                BRVect.dot(swerveConstants.brTurnVect) / BRVect.magnitude())
                / 4.0f
            );

            // Calculate Delta X and Y
            const float deltaX = deltaDistance.x * std::cos(currentPose.rotation);
            const float deltaY = deltaDistance.y * std::sin(currentPose.rotation);

            // Update X, Y, and Rotation
            currentPose.x += deltaX;
            currentPose.y += deltaY;
            currentPose.rotation += deltaRotation;

            // Update Velocity
            updateVelocity(currentPose);
        }

        PoseVelocity getVelocity() override
        {
            return PoseVelocityCalculator::getVelocity();
        }

    private:
        SwerveChassis chassis;
        SwerveConstants swerveConstants;

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        float lastFLDist = 0;
        float lastFRDist = 0;
        float lastBLDist = 0;
        float lastBRDist = 0;
    };
}
