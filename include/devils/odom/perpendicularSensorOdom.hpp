#pragma once

#include "../utils/asyncTask.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/rotationSensor.hpp"
#include "../hardware/gyroBase.h"
#include "poseVelocityCalculator.hpp"

namespace devils
{
    /**
     * Represents an odometry system using a set of perpendicular rotation sensors.
     * If the sensors are parallel, use `ParallelSensorOdometry` instead.
     */
    class PerpendicularSensorOdometry :
        public OdomSource,
        public AsyncTask,
        public PoseVelocityCalculator
    {
    public:
        /**
         * Creates an odometry system using a set of perpendicular rotation sensors.
         * @param verticalSensor The vertical tracking sensor.
         * @param horizontalSensor The horizontal tracking sensor.
         * @param wheelRadius The radius of the wheels in inches.
         */
        PerpendicularSensorOdometry(
            RotationSensor& verticalSensor,
            RotationSensor& horizontalSensor,
            const float wheelRadius)
            : wheelRadius(wheelRadius),
              verticalSensor(verticalSensor),
              horizontalSensor(horizontalSensor)
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

        /**
         * Sets the sensor offsets for the odometry system.
         * Accounts for the difference in sensor placement on the robot.
         * @param newVerticalSensorOffset The offset for the vertical sensor relative to the robot's center of rotation.
         * @param newHorizontalSensorOffset The offset for the horizontal sensor relative to the robot's center of rotation.
         */
        void setSensorOffsets(Vector2& newVerticalSensorOffset,
                              Vector2& newHorizontalSensorOffset)
        {
            this->verticalSensorOffset = &newVerticalSensorOffset;
            this->horizontalSensorOffset = &newHorizontalSensorOffset;
        }

        PoseVelocity getVelocity() override
        {
            return PoseVelocityCalculator::getVelocity();
        }

    protected:
        /**
         * Updates the odometry from vertical and horizontal tracking wheels.
         */
        void onUpdate() override
        {
            // Get Sensor Angles in Degrees
            const float verticalAngle = verticalSensor.getAngle();
            const bool isVerticalSensorError = errno != 0;

            const float horizontalAngle = horizontalSensor.getAngle();
            const bool isHorizontalSensorError = errno != 0;

            // Get Delta Time
            lastUpdateTimestamp = pros::millis();

            // Calculate arc length
            // r * theta
            const float vertical = verticalAngle * wheelRadius;
            const float horizontal = horizontalAngle * wheelRadius;

            // Update IMU
            // Also calculate the change in rotation
            float deltaRotation = 0;
            if (imu != nullptr)
            {
                const auto heading = imu->getHeading();
                if (heading.isSuccess())
                {
                    deltaRotation = heading - lastRotation;
                    lastRotation = heading;
                    currentPose.rotation = heading;
                }
            }

            // Get Delta Distance
            float deltaVertical = vertical - lastVertical;
            float deltaHorizontal = horizontal - lastHorizontal;

            // Check for sensor errors
            if (!isVerticalSensorError)
                lastVertical = vertical;
            if (!isHorizontalSensorError)
                lastHorizontal = horizontal;

            // Apply Sensor Offsets
            if (verticalSensorOffset != nullptr &&
                horizontalSensorOffset != nullptr)
            {
                // Calculate radius of rotation for each sensor
                // We only care about the x and y components since the sensors are perpendicular to the other axis
                const float verticalOffsetRadius = verticalSensorOffset->x;
                const float horizontalOffsetRadius = horizontalSensorOffset->y;

                // Calculate Arc Length
                // Arc Length = r * theta
                const float verticalArcLength = verticalOffsetRadius * deltaRotation;
                const float horizontalArcLength = horizontalOffsetRadius * deltaRotation;

                // Subtract Arc Length
                deltaVertical -= verticalArcLength;
                deltaHorizontal -= horizontalArcLength;
            }

            // Check for sensor errors
            if (isVerticalSensorError)
                deltaVertical = 0;
            if (isHorizontalSensorError)
                deltaHorizontal = 0;

            // Calculate trigonometric values
            const float rotation = currentPose.rotation;
            const float sin = std::sin(rotation);
            const float cos = std::cos(rotation);

            const float deltaX = deltaVertical * cos + deltaHorizontal * sin;
            const float deltaY = deltaVertical * sin - deltaHorizontal * cos;

            // Update Pose
            currentPose.x += deltaX;
            currentPose.y += deltaY;

            // Update Velocity
            updateVelocity(currentPose);
        }

    private:
        const float wheelRadius;
        RotationSensor& verticalSensor;
        RotationSensor& horizontalSensor;

        IGyro* imu = nullptr;

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        float lastVertical = 0;
        float lastHorizontal = 0;
        float lastRotation = 0;

        Vector2* verticalSensorOffset = nullptr;
        Vector2* horizontalSensorOffset = nullptr;
    };
}
