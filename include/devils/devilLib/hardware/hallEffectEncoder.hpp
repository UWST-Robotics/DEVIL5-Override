#pragma once

#include "pros/rotation.hpp"
#include "../geometry/units.hpp"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a Hall Effect Encoder connected to an I2C port. This sensor can be used to measure the rotation of a shaft or wheel.
     * The sensor should be connected to vexbridge
     */
    class HallEffectEncoder : public HardwareBase
    {
    public:
        /**
         * Creates a new Hall Effect Encoder.
         * @param name The name of the sensor (for logging purposes)
         */
        HallEffectEncoder(const std::string& name)
        : HardwareBase(name, "HallEffectEncoder", "I2C port")
        {

        }

        /**
         * Gets the current rotation of the encoder in degrees.
         * @return The current rotation of the encoder in degrees, or an error code if an error occurred.
         */
        HWResult<float> getRotation() const
        {
            //TODO: Implement this funciton using i2c communication to read the rotation from the sensor. The rotation should be returned in degrees.
            reportErrorFromStatus(HWStatus::ERROR_NOT_IMPLEMENTED);
            return HWStatus::ERROR_NOT_IMPLEMENTED;
        }
    };
}
