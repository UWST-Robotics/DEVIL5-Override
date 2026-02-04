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
    class ADIPneumatic : HardwareBase
    {
    public:
        /**
         * Creates a new pneumatic controlled by an ADI port.
         * @param name The name of the pneumatic (for logging purposes)
         * @param port The ADI port of the motor controller (from 1 to 8)
         */
        ADIPneumatic(
            const std::string& name,
            const int8_t port)
            : HardwareBase(name, "ADIPneumatic", port),
              controller(abs(port)),
              isInverted(port < 0)
        {
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

            // Set the value and check for errors
            const auto status = controller.set_value(extended);
            if (status == PROS_ERR)
                reportError();
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
        const pros::adi::DigitalOut controller;

        bool isExtended = false;
        bool isInverted = false;
    };
}
