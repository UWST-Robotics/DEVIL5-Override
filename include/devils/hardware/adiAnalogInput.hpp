#pragma once

#include "pros/adi.hpp"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents an analog (0% - 100%) input from the ADI ports.
     */
    class ADIAnalogInput : ADIHardwareBase
    {
    public:
        /**
         * Creates a new digital input controlled by an ADI port.
         * @param name The name of the input (for logging purposes)
         * @param port The ADI port (from 'A' to 'H').
         * @param isInverted True if the input should be inverted, false otherwise.
         */
        ADIAnalogInput(
            const std::string& name,
            const char port,
            const bool isInverted = false)
            : ADIHardwareBase(name, "ADIAnalogInput", port),
              isInverted(isInverted)
        {
            // ADI Ports need to be configured before they can be used
            executeWithErrorCheck<int32_t>(pros::c::adi_port_set_config, port, pros::E_ADI_ANALOG_IN);
        }

        /**
         * Gets the state of the analog input.
         * @return 1.0f if the input is active, 0.0f if it is not. If the input is inverted, the values are flipped.
         */
        HWResult<float> getValue()
        {
            // Read the value from the ADI port
            const auto result = executeWithErrorCheck<int32_t>(pros::c::adi_analog_read, port);
            if (!result.isSuccess()) return result.status;

            // Invert the value if necessary
            auto value = static_cast<float>(result.value) / MAX_ANALOG_VALUE;
            if (isInverted)
                value = 1.0f - value;

            return value;
        }

    private:
        constexpr static int32_t MAX_ANALOG_VALUE = 4095; // 12-bit maximum value

        bool isInverted = false;
    };
}
