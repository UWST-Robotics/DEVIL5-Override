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
    class OpticalSensor : V5HardwareBase
    {
    public:
        /**
         * Creates a new Optical Sensor.
         * @param name The name of the Optical Sensor (for logging purposes)
         * @param port The port of the Optical Sensor (from 1 to 21)
         */
        OpticalSensor(
            const std::string& name,
            const uint8_t port)
            : V5HardwareBase(name, "OpticalSensor", port)
        {
        }

        /**
         * Sets the brightness of the LED on the Optical Sensor.
         * @param brightness The brightness of the LED from 0 to 1
         */
        void setLEDBrightness(const float brightness)
        {
            // Scale the brightness from [0, 1] to [0, 255] and clamp it to the valid range
            const auto brightnessScaled = std::clamp(brightness, 0.0f, 1.0f) * 255.0f;
            
            // Convert the brightness to an integer
            const auto brightnessInt = static_cast<uint8_t>(brightnessScaled);
            
            // Set the brightness of the LED
            executeWithErrorCheck<int32_t>(pros::c::optical_set_led_pwm, port, brightnessInt);
        }

        /**
         * Gets the current proximity of the Optical Sensor.
         * @return The current proximity of the Optical Sensor as an arbitrary value from 0 to 1 where 1 is the closest.
         */
        HWResult<float> getProximity()
        {
            // Get the proximity from the sensor
            const auto result = executeWithErrorCheck<int32_t>(pros::c::optical_get_proximity, port);
            if (!result.isSuccess())
                return result.status;

            // Scale the proximity from [0, 255] to [0, 1]
            return static_cast<float>(result) / 255.0f;
        }

        /**
         * Gets the current hue of the Optical Sensor.
         * @return The current hue of the Optical Sensor as a value from 0 to 360.
         */
        HWResult<float> getHue()
        {
            // Get the hue from the sensor
            const auto result = executeWithErrorCheck<double>(pros::c::optical_get_hue, port);
            if (!result.isSuccess())
                return result.status;

            // Convert the hue to a float
            return static_cast<float>(result);
        }

        /**
         * Gets the current saturation of the Optical Sensor.
         * @return The current saturation of the Optical Sensor as a percentage value from 0 to 1.
         */
        HWResult<float> getSaturation()
        {
            // Get the saturation from the sensor
            const auto result = executeWithErrorCheck<double>(pros::c::optical_get_saturation, port);
            if (!result.isSuccess())
                return result.status;
            
            // Convert the saturation to a float
            return static_cast<float>(result);
        }

        /**
         * Gets the current brightness of the Optical Sensor.
         * @return The current brightness of the Optical Sensor as a percentage value from 0 to 1.
         */
        HWResult<float> getBrightness()
        {
            // Get the brightness from the sensor
            const auto result = executeWithErrorCheck<double>(pros::c::optical_get_brightness, port);
            if (!result.isSuccess())
                return result.status;
            
            // Convert the brightness to a float
            return static_cast<float>(result);
        }
    };
}
