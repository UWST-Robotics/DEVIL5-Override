#pragma once
#include "pros/optical.hpp"
#include "pros/error.h"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 optical sensor unit.
     */
    class OpticalSensor : private HardwareBase
    {
    public:
        /**
         * Creates a new Optical Sensor.
         * @param name The name of the Optical Sensor (for logging purposes)
         * @param port The port of the Optical Sensor (from 1 to 21)
         */
        OpticalSensor(
            const std::string& name,
            const int8_t port)
            : HardwareBase(name, "OpticalSensor", port),
              sensor(port)
        {
        }

        /**
         * Sets the brightness of the LED on the Optical Sensor.
         * @param brightness The brightness of the LED from 0 to 1
         */
        void setLEDBrightness(const float brightness)
        {
            const auto brightnessInt = static_cast<uint8_t>(std::clamp(brightness, 0.0f, 1.0f) * 255.0f);
            const auto result = sensor.set_led_pwm(brightnessInt);
            if (result == PROS_ERR)
                reportError();
        }

        /**
         * Gets the current proximity of the Optical Sensor.
         * @return The current proximity of the Optical Sensor as an arbitrary value from 0 to 1 where 1 is the closest.
         */
        HWResult<float> getProximity()
        {
            const auto proximity = sensor.get_proximity();
            if (proximity == PROS_ERR)
                return getStatusCode();
            return static_cast<float>(proximity) / 255.0f;
        }

        /**
         * Gets the current hue of the Optical Sensor.
         * @return The current hue of the Optical Sensor as a value from 0 to 360.
         */
        HWResult<float> getHue()
        {
            const auto hue = static_cast<float>(sensor.get_hue());
            if (hue == PROS_ERR_F)
                return getStatusCode();
            return hue;
        }

        /**
         * Gets the current saturation of the Optical Sensor.
         * @return The current saturation of the Optical Sensor as an percentage value from 0 to 1.
         */
        HWResult<float> getSaturation()
        {
            const auto saturation = static_cast<float>(sensor.get_saturation());
            if (saturation == PROS_ERR_F)
                return getStatusCode();
            return saturation;
        }

        /**
         * Gets the current brightness of the Optical Sensor.
         * @return The current brightness of the Optical Sensor as an percentage value from 0 to 1.
         */
        HWResult<float> getBrightness()
        {
            const auto brightness = static_cast<float>(sensor.get_brightness());
            if (brightness == PROS_ERR_F)
                return getStatusCode();
            return brightness;
        }

    private:
        pros::Optical sensor;
    };
}
