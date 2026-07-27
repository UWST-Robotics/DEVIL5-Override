#pragma once

#include "devils/devilLib/devils.h"

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
        void setClawClosed(bool extended)
        {
            claw.setExtended(extended);
        }

        bool getClawState(){
            return claw.getExtended();
        }

    private:
        ADIPneumaticGroup& claw;
    };
}
