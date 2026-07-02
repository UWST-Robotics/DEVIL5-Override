#pragma once

#include <string>
#include "vector2.hpp"

namespace devils
{
    /**
     * Represents a pose in 2D space.
     * Coordinate system convention is red alliance on the left, 0deg to the right, positive is clockwise. Which is weird Austin.
     */
    struct Pose : Vector2
    {
        /// @brief The x position of the robot in inches
        float& x = Vector2::x;

        /// @brief The y position of the robot in inches
        float& y = Vector2::y;

        /// @brief The rotation of the robot in radians
        float rotation = 0;

        /**
         * Constructs a pose with all values set to 0
         */
        Pose() = default;

        /**
         * Constructs a pose with the given x and y
         * @param x The x position of the robot in inches
         * @param y The y position of the robot in inches
         */
        Pose(
            const float x,
            const float y) : Vector2(x, y)
        {
        }

        /**
         * Constructs a pose with the given x, y, and rotation
         * @param x The x position of the robot in inches
         * @param y The y position of the robot in inches
         * @param rotation The rotation of the robot in radians
         */
        Pose(
            const float x,
            const float y,
            const float rotation
        ) : Vector2(x, y),
            rotation(rotation)
        {
        }

        /**
         * Constructs a pose by copying another vector. Sets rotation to 0.
         * @param other The other vector
         */
        Pose(const Vector2& other) : Vector2(other.x, other.y)
        {
        }

        /**
         * Copy constructor
         * @param other The other pose
         */
        Pose(const Pose& other) : Vector2(other.x, other.y), rotation(other.rotation)
        {
        }

        /**
         * Constructs a pose by copying another pose
         * @param other The other pose
         */
        Pose& operator=(const Pose& other)
        {
            x = other.x;
            y = other.y;
            rotation = other.rotation;
            return *this;
        }

        /**
         * Adds two poses together
         * @param other The other pose
         * @return The sum of the two poses
         */
        Pose operator+(const Pose& other) const
        {
            return {x + other.x, y + other.y, rotation + other.rotation};
        }

        /**
         * Subtracts one pose from another
         * @param other The other pose
         * @return The difference of the two poses
         */
        Pose operator-(const Pose& other) const
        {
            return {x - other.x, y - other.y, rotation - other.rotation};
        }

        /**
         * Multiplies a pose by a scalar
         * @param scalar The scalar to multiply by
         * @return The pose multiplied by the scalar
         */
        Pose operator*(const float& scalar) const
        {
            return {x * scalar, y * scalar, rotation * scalar};
        }

        /**
         * Compares two poses for equality
         * @param other The other pose
         * @return True if the poses are equal, false otherwise
         */
        bool operator==(const Pose& other) const
        {
            return x == other.x && y == other.y && rotation == other.rotation;
        }

        /**
         * Compares two poses for inequality
         * @param other The other pose
         * @return True if the poses are not equal, false otherwise
         */
        bool operator!=(const Pose& other) const
        {
            return !(*this == other);
        }

        /**
         * Normalizes the pose
         * @return The normalized pose
         */
        Pose normalize() const
        {
            const float mag = magnitude();
            return {x / mag, y / mag, rotation};
        }

        /**
         * Prints the pose to a string
         * @return The pose as a string
         */
        std::string toString() const
        {
            return "Pose(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(rotation) + ")";
        }

        /**
         * Checks if this pose is behind another pose.
         * This is determined by taking the dot product of the current pose's direction vector and the vector from the current pose to the other pose. If the dot product is negative, then the other pose is behind the current pose.
         * @param otherPose - The other pose to compare to
         * @return True if the other pose is behind this pose, false otherwise
         */
        bool isBehind(const Pose& otherPose) const
        {
            const float dotProduct = cosf(rotation) * (otherPose.x - x) + 
                                     sinf(rotation) * (otherPose.y - y);
            return dotProduct > 0;
        }

        /**
         * Calculates the curvature between this pose and another pose.
         * Curvature is defined as the change in rotation divided by the distance between the two poses.
         * A higher curvature indicates a sharper turn, while a lower curvature indicates a gentler turn.
         * Negative curvature indicates a turn to the left, while positive curvature indicates a turn to the right.
         * @param otherPose - The other pose to compare to
         * @return The curvature between this pose and the other pose (measured in radians per inch)
         */
        float curvature(const Pose& otherPose) const
        {
            const float distance = distanceTo(otherPose);
            if (distance == 0)
                return 0; // Avoid division by zero
            
            const float angleDifference = otherPose.rotation - rotation;
            return angleDifference / distance;
        }
    };

    /**
     * A list of poses to play in sequence
     */
    typedef std::vector<Pose> PoseSequence;
}
