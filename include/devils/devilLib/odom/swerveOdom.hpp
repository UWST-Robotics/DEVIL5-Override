#pragma once
#include "../geometry/pose.hpp"
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
        SwerveOdomOdom(SwerveChassis chassis) : chassis(chassis)
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

            FLdelta = chassis.getWheelPositionInfo(0)[1] - lastFLDist;
            FLVect = Vector2(FLDelta * Math.cos(chassis.getWheelPositionInfo(0)[0]), FLDelta * Math.cos(chassis.getWheelPositionInfo(0)[0]))

            FLrot = FLVect


            // Calculate Delta Distance
            const float deltaDistance = 
            const float deltaRotation = 

            // Calculate Delta X and Y
            const float deltaX = deltaDistance * std::cos(currentPose.rotation + deltaRotation / 2);
            const float deltaY = deltaDistance * std::sin(currentPose.rotation + deltaRotation / 2);

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

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        float lastFLDist = 0;
        float lastFRDist = 0;
        float lastBLDist = 0;
        float lastBRist = 0;
    };
}
