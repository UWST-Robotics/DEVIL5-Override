#pragma once

#include "pros/adi.hpp"
#include "hardwareBase.hpp"
#include <string>
#include <utility>

namespace devils
{
    /**
     * Represents a digital (on/off) input from the ADI ports.
     */
    class ADIDigitalInput : ADIHardwareBase
    {
    public:
        /**
         * Creates a new digital input controlled by an ADI port.
         * @param name The name of the input (for logging purposes)
         * @param port The ADI port (from 'A' to 'H').
         * @param isInverted True if the input should be inverted, false otherwise.
         */
        ADIDigitalInput(
            const std::string& name,
            const char port,
            const bool isInverted = false)
            : ADIHardwareBase(name, "ADIDigitalInput", port),
              isInverted(isInverted)
        {
            // ADI Ports need to be configured before they can be used
            executeWithErrorCheck<int32_t>(pros::c::adi_port_set_config, port, pros::E_ADI_DIGITAL_IN);
        }

        /**
         * Gets the state of the digital input.
         * @return True if the input is high, false if the input is low.
         */
        HWResult<bool> getValue()
        {
            // Read the value from the ADI port
            const auto result = executeWithErrorCheck<bool>(pros::c::adi_digital_read, port);
            if (!result.isSuccess()) return result.status;

            // Invert the value if necessary
            bool value = result.value;
            if (isInverted)
                value = !value;

            return value;
        }

    private:
        bool isInverted = false;
    };
}
