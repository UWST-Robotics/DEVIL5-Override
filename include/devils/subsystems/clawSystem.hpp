#pragma once

#include "devils/devils.h"

namespace devils
{
    class ClawSystem
    {
    public:
        ClawSystem(ADIPneumaticGroup& clawPneumatics, ADIPneumaticGroup& clawFlipPneumatics, 
            ADIPneumaticGroup& doorPneumatics)
            :claw(clawPneumatics), clawFlip(clawFlipPneumatics), door(doorPneumatics)
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

        /**
         * Flips the claw between its normal and flipped positions.
         * @param flipped - True to flip the claw, false to return it to its normal position.
         */
        void setClawFlipped(const bool flipped) const
        {
            clawFlip.setExtended(flipped);
        }

        /**
         * Closes and opens the door
         * @param extended - True to close the door, false to open it.
         */
        void setDoorClosed(const bool extended) const
        {
            door.setExtended(extended);
        }

    private:
        ADIPneumaticGroup& claw;
        ADIPneumaticGroup& clawFlip;
        ADIPneumaticGroup& door;
    };
}
