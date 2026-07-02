#pragma once
#include "../hardware/rotationSensor.hpp"
#include "../geometry/pose.hpp"
#include "odomSource.hpp"
#include "poseVelocityCalculator.hpp"
#include "pros/rtos.hpp"
#include <cmath>

#define M_PI 3.14159265358979323846

namespace devils
{
    /**
     * Represents an odometry system using a set of differential wheels
     */
    class DifferentialWheelOdom : public OdomSource, public PoseVelocityCalculator
    {
    public:
        /**
         * Creates a new odometry system using a set of 2 differential wheels.
         * Position is calculated using the left and right rotations.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        DifferentialWheelOdom(const float wheelRadius,
                              const float wheelBase)
            : wheelRadius(wheelRadius),
              wheelBase(wheelBase)
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
         * Updates the odometry from the left and right rotational values.
         * @param leftRotations The left wheel rotations.
         * @param rightRotations The right wheel rotations.
         */
        void update(
            const float leftRotations,
            const float rightRotations)
        {
            // Get Delta Time
            lastUpdateTimestamp = pros::millis();

            // Get Distance
            const float left = leftRotations * 2 * M_PIF * wheelRadius;
            const float right = rightRotations * 2 * M_PIF * wheelRadius;

            // Get Delta Distance
            const float deltaLeft = left - lastLeft;
            const float deltaRight = right - lastRight;
            lastLeft = left;
            lastRight = right;

            // Calculate Delta Distance
            const float deltaDistance = (deltaLeft + deltaRight) / 2;
            const float deltaRotation = (deltaLeft - deltaRight) / wheelBase;

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
        const float wheelRadius;
        const float wheelBase;

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        float lastLeft = 0;
        float lastRight = 0;
    };
}
