#pragma once

#include "devils/devils.h"

namespace devils
{
    class TubeSystem
    {
    public:
        TubeSystem(ADIPneumatic& tubePneumatics)
            : tube(tubePneumatics)
        {
        }

        /**
         * Raises and lowers the tube
         * @param extended - True to raise the tube, false to lower it.
         */
        void setTubeRaised(const bool extended) const
        {
            tube.setExtended(extended);
        }

    private:
        ADIPneumatic& tube;
    };
}
