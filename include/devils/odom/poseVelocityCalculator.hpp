#pragma once

#include "devils/geometry/pose.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    class PoseVelocityCalculator
    {
    public:
        /**
         * Gets current velocity of the robot.
         * @return The current velocity of the robot as a `PoseVelocity`.
         */
        virtual PoseVelocity getVelocity()
        {
            return currentVelocity;
        }

    protected:
        /**
         * Updates the current velocity of the robot.
         * Should be run whenever the current `pose` is updated.
         * @param pose The current pose of the robot.
         */
        void updateVelocity(const Pose& pose)
        {
            // Calculate Time Delta
            const uint32_t timestamp = pros::millis();
            const auto dt = static_cast<float>(timestamp - lastTimestamp) / 1000.0f;

            // Initial update
            if (lastTimestamp == 0)
            {
                lastTimestamp = timestamp;
                lastPose = pose;
                return;
            }

            // Skip if no time has passed
            if (dt <= 0)
                return;

            // Update Timestamp
            lastTimestamp = timestamp;

            // Calculate Linear Velocity
            const Vector2 positionDelta = pose - lastPose;
            const float linearVelocityX = positionDelta.x / dt;
            const float linearVelocityY = positionDelta.y / dt;

            // Calculate Angular Velocity
            const float angleDelta = pose.rotation - lastPose.rotation;
            const float angularVelocity = angleDelta / dt;

            // Update Velocity
            currentVelocity = PoseVelocity(linearVelocityX, linearVelocityY, angularVelocity);

            // Update Last Pose
            lastPose = pose;
        }

    private:
        // Current state
        PoseVelocity currentVelocity = PoseVelocity();

        // Last state
        uint32_t lastTimestamp = 0;
        Pose lastPose = Pose();
    };
}
