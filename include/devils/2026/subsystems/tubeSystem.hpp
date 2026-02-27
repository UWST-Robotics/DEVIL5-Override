#pragma once

#include "devils/devils.h"
#include <algorithm>
#include <utility>

namespace devils
{
    class TubeSystem
    {
    public:
        TubeSystem(ADIPneumatic tubePneumatics)
            : tube(std::move(tubePneumatics))
        {
        }

        void setTubeRaised(const bool extended)
        {
            tube.setExtended(extended);
        }

    private:
        ADIPneumatic tube;
    };
}
