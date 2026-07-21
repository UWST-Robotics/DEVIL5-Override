#pragma once
#include <string>
#include <cmath>

namespace devils
{
    /**
     * Represents a 3D vector.
     */
    struct Vector2
    {
        /// @brief The x position.
        float x = 0;
        /// @brief The y position.
        float y = 0;

        /**
         * Constructs a 3D vector with all values set to 0
         */
        Vector2() = default;

        /**
         * Constructs a vector with the given x, and y
         * @param x The x position
         * @param y The y position
         */
        Vector2(const float x, const float y) : x(x), y(y)
        {
        }

        /**
         * Copy constructor
         * @param other The other vector
         */
        Vector2(const Vector2& other) = default;

        /**
         * Constructs a vector by copying another vector
         * @param other The other vector
         */
        Vector2& operator=(const Vector2& other) = default;

        /**
         * Adds two vectors together
         * @param other The other vector
         * @return The sum of the two vectors
         */
        Vector2 operator+(const Vector2& other) const
        {
            return {x + other.x, y + other.y};
        }

        /**
         * Subtracts one vector from another
         * @param other The other vector
         * @return The difference of the two vectors
         */
        Vector2 operator-(const Vector2& other) const
        {
            return {x - other.x, y - other.y};
        }

        /**
         * Multiplies a vector by a scalar
         * @param scalar The scalar to multiply by
         * @return The vector multiplied by the scalar
         */
        Vector2 operator*(const float& scalar) const
        {
            return {x * scalar, y * scalar};
        }

        /**
         * Compares two vectors for equality
         * @param other The other vector
         * @return True if the vectors are equal, false otherwise
         */
        bool operator==(const Vector2& other) const
        {
            return x == other.x && y == other.y;
        }

        /**
         * Compares two vectors for inequality
         * @param other The other vector
         * @return True if the vectors are not equal, false otherwise
         */
        bool operator!=(const Vector2& other) const
        {
            return !(*this == other);
        }

        /**
         * Calculates the dot product of two vectors
         * @param other The other vector
         * @return The dot product of the two vectors
         */
        float dot(const Vector2& other) const
        {
            return x * other.x + y * other.y;
        }

        /**
         * Calculates the distance between two vectors
         * @param other The other vector
         * @return The distance between the two vectors
         */
        float distanceTo(const Vector2& other) const
        {
            const float deltaX = x - other.x;
            const float deltaY = y - other.y;

            return std::sqrt(
                deltaX * deltaX +
                deltaY * deltaY);
        }

        /**
         * Calculates the magnitude of the vector
         * @return The magnitude of the vector
         */
        float magnitude() const
        {
            return std::sqrt(
                x * x +
                y * y);
        }

        /**
         * Calculates the angle of the vector to the x-axis
         * @return The angle of the vector in radians
         */
        float angleToX() const
        {
            return atan2(y,x);
        }

        /**
         * Normalizes the vector
         * @return The normalized vector
         */
        Vector2 normalize() const
        {
            const float mag = magnitude();
            return {x / mag, y / mag};
        }

        /**
         * Rotates the vector, positive is counterclockwise
         * @return The rotated vector
         */
        Vector2 rotate(const float& angle) const
        {
            return {x*cos(angle) - y*sin(angle), x*sin(angle + y*cos(angle))};
        }

        /**
         * Prints the vector to a string
         * @return The vector as a string
         */
        std::string toString() const
        {
            return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
        }
    };
}
