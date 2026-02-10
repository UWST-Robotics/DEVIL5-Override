#pragma once

#include "pros/adi.hpp"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a non-V5 pneumatic valve controlled by ADI ports.
     * See https://github.com/msoe-vex/pcb-design/tree/main/VEX%20Solenoid%20Driver%20V2%20Complete
     */
    class ADIPneumatic : ADIHardwareBase
    {
    public:
        /**
         * Creates a new pneumatic controlled by an ADI port.
         * @param name The name of the pneumatic (for logging purposes)
         * @param port The ADI port (from 'A' to 'H').
         * @param isInverted True if the pneumatic should be inverted, false otherwise.
         */
        ADIPneumatic(
            const std::string& name,
            const char port,
            const bool isInverted = false)
            : ADIHardwareBase(name, "ADIPneumatic", port),
              isInverted(isInverted)
        {
            // ADI Ports need to be configured before they can be used
            executeWithErrorCheck<int32_t>(pros::c::adi_port_set_config, port, pros::E_ADI_DIGITAL_OUT);
        }

        /**
         * Sets the state of the pneumatic.
         * @param extended True to extend the pneumatic, false to retract it.
         */
        void setExtended(bool extended)
        {
            // Invert the value if the port is inverted
            if (isInverted)
                extended = !extended;
            this->isExtended = extended;

            // Write the value to the ADI port
            executeWithErrorCheck<int32_t>(pros::c::adi_digital_write, port, extended);
        }

        /**
         * Extends the pneumatic.
         */
        void extend()
        {
            setExtended(true);
        }

        /**
         * Retracts the pneumatic.
         */
        void retract()
        {
            setExtended(false);
        }

        /**
         * Checks if the pneumatic is extended.
         * @return True if the pneumatic is extended, false otherwise.
         */
        bool getExtended() const
        {
            return isExtended;
        }

    private:
        bool isExtended = false;
        bool isInverted = false;
    };
}
