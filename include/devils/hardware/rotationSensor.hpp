#pragma once

#include "pros/rotation.hpp"
#include "../geometry/units.hpp"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a VEX V5 rotational sensor.
     * https://www.vexrobotics.com/276-6050.html
     */
    class RotationSensor : public V5HardwareBase
    {
    public:
        /**
         * Creates a new Rotation Sensor.
         * @param name The name of the rotational sensor (for logging purposes)
         * @param port The port of the rotational sensor (from 1 to 21). Negative ports are reversed.
         */
        RotationSensor(
            const std::string& name,
            const int8_t port)
            : V5HardwareBase(name, "RotationSensor", abs(port))
        {
        }

        /**
         * Sets the position of the sensor in degrees.
         * @param position The position to set the sensor to.
         */
        void setPosition(const float position)
        {
            // Convert the position to centidegrees
            auto centidegPosition = static_cast<int32_t>(Units::degToCentideg(position));

            // Invert the position if necessary
            if (isInverted)
                centidegPosition = -centidegPosition;

            // Write the position to the sensor
            executeWithErrorCheck<int32_t>(pros::c::rotation_set_position, port, centidegPosition);
        }

        /**
         * Gets the absolute angle of the sensor in degrees (rounded to the nearest centidegree)
         * @return The absolute angle of the sensor in degrees.
         */
        HWResult<float> getAngle()
        {
            // Get the angle from the sensor
            const auto result = executeWithErrorCheck<int32_t>(pros::c::rotation_get_position, port);
            if (!result.isSuccess())
                return result.status;

            // Invert the angle if necessary
            auto angle = static_cast<float>(result.value);
            if (isInverted)
                angle = -angle;

            // Convert from centidegrees to degrees and return
            return Units::centidegToDeg(angle);
        }

        /**
         * Gets the velocity of the sensor in degrees per second.
         * @return The velocity of the sensor in degrees per second.
         */
        HWResult<float> getVelocity()
        {
            // Get the velocity from the sensor
            const auto result = executeWithErrorCheck<int32_t>(pros::c::rotation_get_velocity, port);
            if (!result.isSuccess())
                return result.status;

            // Invert the velocity if necessary
            auto velocity = static_cast<float>(result.value);
            if (isInverted)
                velocity = -velocity;

            return Units::centidegToDeg(velocity);
        }

    protected:
        bool isInverted = false;
    };
}
