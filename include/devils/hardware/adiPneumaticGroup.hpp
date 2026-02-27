#pragma once

#include <string>
#include <utility>
#include "adiPneumatic.hpp"

namespace devils
{
    typedef std::vector<std::shared_ptr<ADIPneumatic>> ADIPneumaticList;

    /**
     * Represents a set of `ADIPneumatic`s grouped together.
     */
    class ADIPneumaticGroup
    {
    public:
        /**
         * Creates a new ADI pneumatic group.
         * @param name The name of the pneumatic group (for logging purposes)
         * @param ports The ADI ports of the pneumatics in the group (from 'A' to 'H').
         * @param isInverted True if the pneumatics should be inverted, false otherwise.
         */
        ADIPneumaticGroup(
            std::string name,
            const std::initializer_list<char> ports,
            bool isInverted = false)
            : name(std::move(name))
        {
            pneumatics.reserve(ports.size());
            for (auto port : ports)
                pneumatics.push_back(std::make_shared<ADIPneumatic>(
                    getPneumaticName(port),
                    port,
                    isInverted));
        }

        /**
         * Sets the state of all the pneumatics in the group.
         * @param isExtended True to extend the pneumatics, false to retract them.
         */
        void setExtended(const bool isExtended) const
        {
            for (const auto& pneumatic : pneumatics)
                pneumatic->setExtended(isExtended);
        }

        /**
         * Extends all the pneumatics in the group.
         */
        void extend() const
        {
            setExtended(true);
        }

        /**
         * Retracts all the pneumatics in the group.
         */
        void retract() const
        {
            setExtended(false);
        }

        /**
         * Checks if all the pneumatics in the group are extended.
         * @return True if all the pneumatics in the group are extended, false otherwise.
         */
        bool getExtended() const
        {
            for (const auto& pneumatic : pneumatics)
                if (!pneumatic->getExtended())
                    return false;
            return true;
        }

        /**
         * Gets the pneumatics in the motor group.
         * @return The pneumatics in the motor group.
         */
        ADIPneumaticList& getPneumatics()
        {
            return pneumatics;
        }

    private:
        /**
         * Gets the name of each pneumatic in the pneumatic group.
         * @param port The port of the pneumatic
         * @return The name of the pneumatic
         */
        std::string getPneumaticName(const char port) const
        {
            return name + "_" + ADIHardwareBase::adiPortToString(port);
        }

        const std::string name;
        ADIPneumaticList pneumatics;
    };
}
