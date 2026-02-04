#pragma once

#include "../autoStep.hpp"
#include "../../utils/timer.hpp"

namespace devils
{
    /**
     * Pauses the autonomous routine for a given duration.
     */
    class AutoPauseStep : public AutoStep
    {
    public:
        /**
         * Pauses the autonomous routine for a given duration.
         * @param duration The duration to pause in milliseconds.
         */
        AutoPauseStep(const uint32_t duration)
            : timer(duration)
        {
        }

    protected:
        void onStart() override
        {
            timer.start();
        }

        bool checkFinished() override
        {
            return timer.finished();
        }

        Timer timer;
    };
}
