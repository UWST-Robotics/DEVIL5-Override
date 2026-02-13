#pragma once

namespace devils
{
    /**
     * Represents various constraints for generating a trajectory.
     * Defines how fast the robot can move and accelerate over time.
     */
    struct TrajectoryConstraints
    {
        /// @brief The maximum velocity of the robot in inches per second
        float maxVelocity = 36.0f; // in/s

        /// @brief The maximum acceleration of the robot in inches per second squared
        float maxAcceleration = 48.0f; // in/s^2

        /// @brief The maximum deceleration of the robot in inches per second squared
        float maxDeceleration = 100.0f; // in/s^2
        
        /// @brief The default constraints for trajectory generation
        static TrajectoryConstraints defaultConstraints;
    };
}
