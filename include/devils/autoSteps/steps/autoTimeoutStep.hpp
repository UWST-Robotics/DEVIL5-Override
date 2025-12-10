#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../utils/timer.hpp"

namespace devils
{
    /**
     * Automatically starts and stops a step during a fixed duration.
     */
    class AutoTimeoutStep : public AutoStep
    {
    public:
        /**
         * Creates a new timeout step.
         * @param autoStep The step to manage.
         * @param duration The duration of the step in milliseconds.
         */
        AutoTimeoutStep(
            AutoStepPtr autoStep,
            uint32_t duration)
            : autoStep(autoStep),
              timer(duration)
        {
        }

    protected:
        void onStart() override
        {
            // Start Timer
            timer.start();

            // Start Auto Step
            if (autoStep)
                autoStep->start();
        }

        void onUpdate() override
        {
            // Check if the Auto Step has finished early
            if (autoStep && autoStep->getState() == AsyncTaskState::STOPPED)
                stop();
        }

        void onStop() override
        {
            // Stop Timer
            timer.stop();

            // Stop Auto Step
            if (autoStep)
                autoStep->stop();
        }

        bool checkFinished() override
        {
            // Check Timer
            return timer.finished();
        }

    protected:
        // Params
        AutoStepPtr autoStep;
        Timer timer;
    };
}