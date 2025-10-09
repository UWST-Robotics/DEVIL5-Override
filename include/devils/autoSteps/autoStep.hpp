#pragma once

#include "../utils/asyncTask.hpp"
#include <type_traits>
#include <memory>

namespace devils
{
    class AutoStep : public AsyncTask
    {
    public:
        // Allows inheritance by running `AsyncTask` constructor/destructor
        AutoStep() : AsyncTask("AutoStep") {}
        AutoStep(const std::string &debugName) : AsyncTask(debugName) {}
        virtual ~AutoStep() = default;

        /**
         * Stops all running `AutoStep` instance.
         * Should be called on disable to ensure all autonomous steps are stopped cleanly.
         */
        static void stopAll()
        {
            auto allRunningInstances = AsyncTask::getAllRunningInstances();

            // Check each instance to see if it's an AutoStep and stop it if so
            for (auto &instance : allRunningInstances)
            {
                // Use dynamic_pointer_cast to check if the instance is an AutoStep
                if (auto autoStepInstance = std::dynamic_pointer_cast<AutoStep>(instance))
                    autoStepInstance->stop();
            }
        }
    };

    // Define a shared pointer type for AutoStep
    typedef std::shared_ptr<AutoStep> AutoStepPtr;
}
