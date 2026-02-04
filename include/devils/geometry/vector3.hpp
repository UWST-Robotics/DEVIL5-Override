#pragma once

#include <string>
#include <cmath>

namespace devils
{
    /**
     * Represents a 3D vector.
     */
    struct Vector3
    {
        /// @brief The x position.
        float x = 0;
        /// @brief The y position.
        float y = 0;
        /// @brief The z position.
        float z = 0;

        /**
         * Constructs a 3D vector with all values set to 0
         */
        Vector3() = default;

        /**
         * Constructs a vector with the given x, and y
         * @param x The x position
         * @param y The y position
         */
        Vector3(const float x, const float y) : x(x), y(y)
        {
        }

        /**
         * Constructs a vector with the given x, y, and z
         * @param x The x position
         * @param y The y position
         * @param z The z position
         */
        Vector3(const float x, const float y, const float z) : x(x), y(y), z(z)
        {
        }

        /**
         * Copy constructor
         * @param other The other vector
         */
        Vector3(const Vector3& other) = default;

        /**
         * Constructs a vector by copying another vector
         * @param other The other vector
         */
        Vector3& operator=(const Vector3& other) = default;

        /**
         * Adds two vectors together
         * @param other The other vector
         * @return The sum of the two vectors
         */
        Vector3 operator+(const Vector3& other) const
        {
            return {x + other.x, y + other.y, z + other.z};
        }

        /**
         * Subtracts one vector from another
         * @param other The other vector
         * @return The difference of the two vectors
         */
        Vector3 operator-(const Vector3& other) const
        {
            return {x - other.x, y - other.y, z - other.z};
        }

        /**
         * Multiplies a vector by a scalar
         * @param scalar The scalar to multiply by
         * @return The vector multiplied by the scalar
         */
        Vector3 operator*(const float& scalar) const
        {
            return {x * scalar, y * scalar, z * scalar};
        }

        /**
         * Compares two vectors for equality
         * @param other The other vector
         * @return True if the vectors are equal, false otherwise
         */
        bool operator==(const Vector3& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }

        /**
         * Compares two vectors for inequality
         * @param other The other vector
         * @return True if the vectors are not equal, false otherwise
         */
        bool operator!=(const Vector3& other) const
        {
            return !(*this == other);
        }

        /**
         * Calculates the dot product of two vectors
         * @param other The other vector
         * @return The dot product of the two vectors
         */
        float dot(const Vector3& other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        /**
         * Calculates the distance between two vectors
         * @param other The other vector
         * @return The distance between the two vectors
         */
        float distanceTo(const Vector3& other) const
        {
            const float deltaX = other.x - x;
            const float deltaY = other.y - y;
            const float deltaZ = other.z - z;

            return std::sqrtf(
                deltaX * deltaX +
                deltaY * deltaY +
                deltaZ * deltaZ);
        }

        /**
         * Calculates the magnitude of the vector
         * @return The magnitude of the vector
         */
        float magnitude() const
        {
            return std::sqrtf(
                x * x +
                y * y +
                z * z);
        }

        /**
         * Normalizes the vector
         * @return The normalized vector
         */
        Vector3 normalize() const
        {
            const float mag = magnitude();
            return {x / mag, y / mag, z / mag};
        }

        /**
         * Prints the vector to a string
         * @return The vector as a string
         */
        std::string toString() const
        {
            return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
        }
    };
}
