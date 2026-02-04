#pragma once
#include "pros/rotation.hpp"
#include "../geometry/units.hpp"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 rotational sensor.
     */
    class RotationSensor : HardwareBase
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
            : HardwareBase(name, "RotationSensor", port),
              rotationSensor(port)
        {
        }

        /**
         * Sets the position of the sensor in centidegrees.
         * @param position The position to set the sensor to.
         */
        void setPosition(const uint32_t position) const
        {
            const auto status = rotationSensor.set_position(position);
            if (status == PROS_ERR)
                reportError();
        }

        /**
         * Gets the absolute angle of the sensor in radians.
         * @return The absolute angle of the sensor in radians.
         */
        HWResult<float> getAngle() const
        {
            const auto angle = rotationSensor.get_position();
            if (angle == PROS_ERR)
                return getStatusCode();
            return Units::centidegToRad(static_cast<float>(angle));
        }

        /**
         * Gets the velocity of the sensor in radians per second.
         * @return The velocity of the sensor in radians per second.
         */
        HWResult<float> getVelocity() const
        {
            const auto velocity = rotationSensor.get_velocity();
            if (velocity == PROS_ERR)
                return getStatusCode();
            return Units::centidegToRad(static_cast<float>(velocity));
        }

    private:
        pros::Rotation rotationSensor;
    };
}
