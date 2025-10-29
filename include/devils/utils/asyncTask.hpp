#pragma once
#include "pros/rtos.hpp"
#include <memory>
#include "logger.hpp"

namespace devils
{
    /// @brief The current state of an `AsyncTask`.
    enum AsyncTaskState
    {
        // `onStart` is currently executing
        STARTING,

        // The task is running and `onUpdate` is being called every 20ms
        STARTED,

        // `onStop` is currently executing
        STOPPING,

        // The task is not running
        STOPPED,
    };

    /**
     * Represents an asynchronous object that executes asynchronously from the main program.
     * Contains methods for `onStart`, `onUpdate`, `onStop`, and `checkFinished`.
     *
     * \note
     * Any classes derived from `AsyncTask` must be managed by `std::shared_ptr` AND
     * publically inherit from `AsyncTask` to properly up-cast when using `shared_from_this()`.
     */
    class AsyncTask : public std::enable_shared_from_this<AsyncTask>
    {
    public:
        AsyncTask() = default;
        AsyncTask(const std::string &debugName) : AsyncTask()
        {
            setDebugName(debugName);
        }

        ~AsyncTask()
        {
            stop();
        }

        /**
         * If the `AsyncTask` is running, it stops it.
         * Does nothing if `getState()` is not `STARTED`.
         *
         * When stopped, `onStop()` is called once and the task is no longer updated.
         * Use `start()` to start the task again.
         */
        void stop()
        {
            if (state == STOPPING || state == STOPPED)
                return;

            // Set state to STOPPING
            stateMutex.take(1000);
            state = STOPPING;

            // Call onStop
            onStop();

            // Set the state to STOPPED
            state = STOPPED;
            stateMutex.give();
        }

        /**
         * If the `AsyncTask` is not running, it starts it.
         * Does nothing if `getState()` is not `STOPPED`.
         *
         * When started, `onStart()` is called once, and then `onUpdate()` is called every 20ms by a hidden background task.
         * Use `stop()` to stop the task at any time.
         *
         * @return A shared pointer to the current `AsyncTask` instance.
         */
        std::shared_ptr<AsyncTask> start()
        {
            if (state == STARTING || state == STARTED)
                return shared_from_this();

            // Set state to STARTING
            stateMutex.take(1000);
            state = STARTING;

            // Add this task to the all running instances list
            allRunningInstancesMutex.take();
            allRunningInstances.push_back(shared_from_this());
            allRunningInstancesMutex.give();

            // Start the background task if it hasn't been started yet
            if (!backgroundTask)
                backgroundTask = std::make_unique<pros::Task>(prosBackgroundTask, "AsyncTask Background Task");

            // Call onStart
            onStart();

            // Set the state to STARTED
            state = STARTED;
            stateMutex.give();

            return shared_from_this();
        }

        /**
         * Blocks the current thread until the task is fully stopped.
         * If the task is already stopped, returns immediately.
         *
         * @return A shared pointer to the current `AsyncTask` instance.
         */
        std::shared_ptr<AsyncTask> join()
        {
            while (state != STOPPED)
                pros::delay(UPDATE_INTERVAL);

            return shared_from_this();
        }

        /**
         * Gets the current state of the task.
         * @return The current state of the task.
         */
        AsyncTaskState getState() const
        {
            return state;
        }

        /**
         * Gets a list of all active `AsyncTask` instances.
         * @return A vector of weak pointers to all active `AsyncTask` instances.
         */
        static std::vector<std::shared_ptr<AsyncTask>> getAllRunningInstances()
        {
            return allRunningInstances;
        }

    protected:
        /// @brief Called before the task starts.
        virtual void onStart() {}

        /// @brief Called every 20ms to update the task state.
        virtual void onUpdate() {};

        /// @brief Called before the task stops.
        virtual void onStop() {};

        /// @brief Called every update to check if the task is finished. If so, `stop()` is called automatically. Default implementation always returns false.
        /// @return True if the task is finished, false otherwise.
        virtual bool checkFinished() { return false; };

        /// @brief Gets the debug name of the task.
        std::string getDebugName() const
        {
            return debugName;
        }

        /// @brief Sets the debug name of the task.
        void setDebugName(const std::string &name)
        {
            debugName = name;
        }

    private:
        /**
         * The hidden background task that updates all active AsyncTask instances.
         * Started automatically when the first AsyncTask is created.
         */
        static void prosBackgroundTask()
        {
            if (!backgroundTask)
                return;

            while (true)
            {
                // Iterate over all tasks and call their onUpdate method if they are running
                allRunningInstancesMutex.take();
                for (int i = 0; i < allRunningInstances.size(); i++)
                {
                    // Get a shared pointer from the weak pointer
                    auto instance = allRunningInstances[i];

                    try
                    {
                        // Check if the instance is stopped and clean it from the list
                        if (instance->getState() == STOPPED)
                        {
                            allRunningInstances.erase(allRunningInstances.begin() + i);
                            i--;
                            continue;
                        }

                        // Call onUpdate if the instance is started
                        if (instance->getState() == STARTED)
                            instance->onUpdate();

                        // Call checkFinished. Stop the instance if it returns true
                        if (instance->checkFinished())
                            instance->stop();
                    }

                    // Catch any exceptions to prevent the background task from crashing
                    catch (const std::exception &e)
                    {
                        Logger::error("An error occurred in AsyncTask: " + std::string(e.what()));
                    }
                }
                allRunningInstancesMutex.give();

                pros::delay(UPDATE_INTERVAL);
            }
        }

    private:
        static constexpr int UPDATE_INTERVAL = 20; // Default update interval in milliseconds

        // List of all running instances
        static pros::Mutex allRunningInstancesMutex;
        static std::vector<std::shared_ptr<AsyncTask>> allRunningInstances; // <-- Using shared_ptr to ensure instances stay alive while running

        // Background task to update all AsyncTask instances
        static std::unique_ptr<pros::Task> backgroundTask;

        // State of this current task
        pros::Mutex stateMutex;
        AsyncTaskState state = STOPPED;
        std::string debugName = "AsyncTask";
    };

    // Define the static members
    pros::Mutex AsyncTask::allRunningInstancesMutex;
    std::vector<std::shared_ptr<AsyncTask>> AsyncTask::allRunningInstances;
    std::unique_ptr<pros::Task> AsyncTask::backgroundTask;
}