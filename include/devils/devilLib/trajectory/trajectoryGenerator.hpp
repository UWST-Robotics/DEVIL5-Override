#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include "trajectory.hpp"
#include "trajectoryConstraints.hpp"
#include "../path/path.hpp"
#include "../geometry/units.hpp"
#include "../geometry/math.hpp"

namespace devils
{
    /**
     *  Generates a `Trajectory` from a `Path` using a trapezoidal motion profile.
     */
    class TrajectoryGenerator
    {
    public:
        /// @brief Path parameters
        struct PathInfo
        {
            /// @brief The initial velocity in inches per second
            float startingVelocity = 0;

            /// @brief The final velocity in inches per second
            float endingVelocity = 0;
        };

        /**
         * Creates a new trajectory generator.
         * @param constraints The robot constraints
         * @param pathInfo The path information
         */
        explicit TrajectoryGenerator(
            const PathInfo pathInfo,
            const TrajectoryConstraints& constraints = TrajectoryConstraints::defaultConstraints)
            : constraints(constraints),
              pathInfo(pathInfo)
        {
        }

        /**
         * Calculates the trajectory from a path.
         * This is a resource-intensive operation and should be called sparingly.
         *
         * @details This function steps through each point and uses `TrajectoryConstraints`
         *          to constrain the acceleration and velocity of the robot. Then, it runs another pass backwards
         *          to constrain the deceleration and velocity of the robot. Finally, it calculates the time,
         *          actual acceleration, and angular velocity for each point.
         *
         * @param path The path to generate the trajectory from
         * @return The generated trajectory
         */
        std::shared_ptr<Trajectory> calc(Path& path) const
        {
            // The velocities at each step
            auto points = std::make_unique<std::vector<Trajectory::Point>>();
            auto length = path.getLength() / DELTA_INDEX + 1;
            points->reserve(static_cast<size_t>(length) + 1);

            // Initial Robot State
            Trajectory::Point previousPoint = {
                0,
                path.getPoseAt(0),
                pathInfo.startingVelocity,
                0,
                constraints.maxAcceleration
            };
            points->push_back(previousPoint);

            // FORWARD PASS
            // Iterate through the path and constrain the velocity in the acceleration phase
            for (float i = 0; i <= path.getLength() - 1 + DELTA_INDEX; i += DELTA_INDEX)
            {
                // Get pose
                Pose currentPose = path.getPoseAt(i);

                // Calculate distance
                const float deltaDistance = currentPose.distanceTo(previousPoint.pose);

                // Calculate velocity
                // v_f = sqrt(v_i^2 + 2*a*d)
                float velocity = previousPoint.velocity * previousPoint.velocity +
                    2 * constraints.maxAcceleration * deltaDistance;
                velocity = sqrtf(velocity);

                // Clamp velocity to constraints
                velocity = std::min(velocity, constraints.maxVelocity);

                // Clamp velocity to rotational constraints
                const float curvature = std::abs(currentPose.curvature(previousPoint.pose));
                if (curvature > 0)
                {
                    const float maxVelocityFromRotation = Units::degToRad(constraints.rotationalMaxVelocity) /
                        curvature;
                    velocity = std::min(velocity, maxVelocityFromRotation);
                }

                // Calculate dot product to determine if the robot is moving forward or backward
                const float prevDotCurrentPose = cosf(previousPoint.pose.rotation) *
                    (currentPose.x - previousPoint.pose.x) +
                    sinf(previousPoint.pose.rotation) *
                    (currentPose.y - previousPoint.pose.y);

                // If the dot product is negative, we are moving backwards
                if (prevDotCurrentPose < 0)
                {
                    // Reverse the velocity
                    velocity = -velocity;
                }

                // Append to points
                Trajectory::Point point = {
                    0, // Time is calculated later
                    currentPose,
                    velocity,
                    0,
                    0
                };
                points->push_back(point);

                // Update previous point
                previousPoint = point;
            }

            // Ending Robot State
            previousPoint.velocity = pathInfo.endingVelocity;

            // REVERSE PASS
            // Iterate through the path and constrain the velocity in the deceleration phase
            for (int i = static_cast<int>(points->size()) - 1; i >= 0; i--)
            {
                // Grab existing point
                auto& point = points->at(i);

                // Calculate distance
                const float deltaDistance = point.pose.distanceTo(previousPoint.pose);

                // Calculate velocity
                // Note: Uses deceleration instead of acceleration since
                // we are stepping from the end of the path to the beginning
                // v_f = sqrt(v_i^2 + 2*a*d)
                float velocity = previousPoint.velocity * previousPoint.velocity +
                    2 * constraints.maxDeceleration * deltaDistance;
                velocity = sqrtf(velocity);

                // Calculate dot product to determine if the robot is moving forward or backward
                const float prevDotCurrentPose = cosf(point.pose.rotation) *
                    (previousPoint.pose.x - point.pose.x) +
                    sinf(point.pose.rotation) *
                    (previousPoint.pose.y - point.pose.y);

                // If the dot product is negative, we are moving backwards
                if (prevDotCurrentPose < 0)
                    velocity = -velocity;

                // Clamp velocity to existing point
                velocity = Math::smallestMagnitude({velocity, point.velocity});

                // Update previous point
                point.velocity = velocity;
                previousPoint = point;
            }

            // Reset `previousPoint` to the first point
            previousPoint = points->front();

            // FINAL PASS
            // Calculate the time, acceleration, and angular velocity for each point
            for (size_t i = 1; i < points->size(); i++)
            {
                // Get points
                auto& point = points->at(i);

                // Calculate distance
                const float deltaDistance = point.pose.distanceTo(previousPoint.pose);

                // Calculate actual acceleration
                // a = (v_f^2 - v_i^2) / (2 * d)
                float acceleration = point.velocity * point.velocity -
                    previousPoint.velocity * previousPoint.velocity;
                acceleration /= 2 * deltaDistance;

                if (std::isnan(acceleration))
                    acceleration = 0;

                // Calculate delta time
                // 0 = (1/2)at^2 + v_i*t - d
                // t = (-v_i + sqrt(v_i^2 + 2ad)) / a
                float deltaTime = previousPoint.velocity * previousPoint.velocity;
                deltaTime += 2 * acceleration * deltaDistance;
                deltaTime = (-std::abs(previousPoint.velocity) + std::sqrt(deltaTime)) / acceleration;

                // If the time is NaN, acceleration is 0
                // t = d / v
                if (std::isnan(deltaTime))
                    deltaTime = deltaDistance / std::abs(previousPoint.velocity);

                // If the time is NaN, set to 0
                if (std::isnan(deltaTime))
                    deltaTime = 0;

                // Safety check to prevent large jumps in time due to numerical instability
                if (deltaTime > 1.0f)
                    deltaTime = 0.0f; // Cap delta time to prevent large jumps

                // Calculate angular velocity
                float angularVelocity = Units::diffRad(point.pose.rotation, previousPoint.pose.rotation) / deltaTime;
                if (std::isnan(angularVelocity))
                    angularVelocity = 0;

                // Update point
                point.t = previousPoint.t + deltaTime;
                point.acceleration = acceleration;
                point.angularVelocity = angularVelocity;

                // Update previous point
                previousPoint = point;
            }

            return std::make_shared<Trajectory>(std::move(points));
        }

    private:
        /// @brief The step size in path indices
        static constexpr float DELTA_INDEX = 0.01;

        /// @brief Constraints of trajectory generation
        TrajectoryConstraints constraints;

        /// @brief The path information
        PathInfo pathInfo;
    };
}
