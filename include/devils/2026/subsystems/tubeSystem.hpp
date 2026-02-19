#pragma once

#include "devils/devils.h"
#include <algorithm>

namespace devils
{
    /**
     * Represents the intake arm and claw system of the robot.
     */
    class TubeSystem
    {

    public:
        TubeSystem(ADIPneumatic& tubePneumatics, ADIPneumatic& hoodPneumatics)
            : tube(tubePneumatics), hood(hoodPneumatics)
        {

        }

        void setHoodOpen(bool extended)
        {
            hood.setExtended(extended);
        }

        void setTubeRaised(bool extended)
        {
            tube.setExtended(extended);
        }

    private:
        ADIPneumatic hood;
        ADIPneumatic tube;
    };
    
}
