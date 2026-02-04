#pragma once

#include "pros/vision.hpp"
#include "pros/error.h"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a VEX legacy vision sensor object.
     */
    class LegacyVisionSensor : HardwareBase
    {
    public:
        // Thank you James Pearman for these measurements
        // https://www.vexforum.com/t/vision-sensor-fov-measurements/62397
        static constexpr int VISION_WIDTH_PX = VISION_FOV_WIDTH; // px
        static constexpr int VISION_HEIGHT_PX = VISION_FOV_HEIGHT; // px
        static constexpr int VISION_WIDTH_FOV = 61; // degrees
        static constexpr int VISION_HEIGHT_FOV = 41; // degrees

        /**
         * Creates a vision sensor object.
         * @param name The name of the motor (for logging purposes)
         * @param port The port of the motor (from 1 to 21)
         */
        LegacyVisionSensor(
            const std::string& name,
            const int8_t port)
            : HardwareBase(name, "VisionSensor", port),
              sensor(port, pros::E_VISION_ZERO_CENTER)
        {
        }

        /**
         * Sets the vision sensor's LED color.
         * Overrides the default LED behavior.
         * @param color The color to set the LED to
         */
        void setLEDColor(int32_t color)
        {
            const auto status = sensor.set_led(color);
            if (status == PROS_ERR)
                reportError();
        }

        /**
         * Resets the vision sensor's LED color to the default behavior.
         */
        void resetLEDColor() const
        {
            const auto status = sensor.clear_led();
            if (status == PROS_ERR)
                reportError();
        }

        HWResult<bool> hasTargets() const
        {
            const auto objectCount = sensor.get_object_count();
            if (objectCount == PROS_ERR)
                return getStatusCode();
            return objectCount > 0;
        }

    private:
        pros::Vision sensor;
    };
}
