#pragma once

#include "devils/devils.h"

namespace devils
{
    class ClawSystem
    {
    public:
        ClawSystem(ADIPneumaticGroup& clawPneumatics)
            :claw(clawPneumatics)
        {
        }

        /**
         * Closes and opens the claw
         * @param extended - True to close the claw, false to open it.
         */
        void setClawClosed(const bool extended) const
        {
            claw.setExtended(extended);
        }

    private:
        ADIPneumaticGroup& claw;
    };
}
