#pragma once

#include "devils/devils.h"

namespace devils
{
    class WingSystem
    {
    public:
        WingSystem(ADIPneumatic& wingPneumatics)
            :wing(wingPneumatics)
        {
        }

        /**
         * Raises and lowers the wing
         * @param extended - True to raise the wing, false to lower it.
         */
        void setWingRaised(const bool extended) const
        {
            wing.setExtended(extended);
        }

    private:
        ADIPneumatic& wing;
    };
}
