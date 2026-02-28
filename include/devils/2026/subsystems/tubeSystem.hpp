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

        void setTubeRaised(const bool extended) const
        {
            tube.setExtended(extended);
        }

    private:
        ADIPneumatic& tube;
    };
}
