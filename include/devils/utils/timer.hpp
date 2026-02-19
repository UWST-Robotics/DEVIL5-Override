#pragma once
#include "stopwatch.hpp"

namespace devils
{
    /**
     * Represents a timer that can be used to measure elapsed time against a specified duration.
     * The timer can be started and stopped, and the elapsed time and time remaining can be retrieved.
     * The timer is considered finished when the elapsed time exceeds the specified duration.
     */
    class Timer
    {
    public:
        /**
         * Creates a new instance of Timer with a given duration.
         * By default, the timer is automatically started.
         * It can be restarted by calling the `start()` method again.
         * @param duration The duration of the timer in seconds.
         * @return The new instance of Timer.
         */
        explicit Timer(const float duration) :
            duration(duration)
        {
        }

        /**
         * Sets the duration of the timer.
         * @param newDuration The duration of the timer in seconds.
         */
        void setDuration(const float newDuration)
        {
            this->duration = newDuration;
        }

        /**
         * Starts the timer.
         */
        void start()
        {
            stopwatch.start();
        }

        /**
         * Stops the timer.
         */
        void stop()
        {
            stopwatch.stop();
        }

        /**
         * Gets whether the timer has started.
         * @return True if the timer has started, false otherwise.
         */
        bool getIsRunning() const
        {
            return stopwatch.getIsRunning() && !getIsFinished();
        }

        /**
         * Gets whether the timer has finished.
         * @return True if the timer has finished, false otherwise.
         */
        bool getIsFinished() const
        {
            return stopwatch.getIsRunning() && getElapsedTime() >= duration;
        }

        /**
         * Gets the elapsed time since the timer was started.
         * @return The elapsed time since the timer was started in seconds.
         */
        float getElapsedTime() const
        {
            return stopwatch.getTime();
        }
        
        /**
         * Gets the time remaining on the timer.
         * @return The time remaining on the timer in seconds.
         */
        float getTimeRemaining() const
        {
            if (!getIsRunning())
                return 0;
            return duration - getElapsedTime();
        }

    protected:
        Stopwatch stopwatch;
        float duration = 0;
    };
}
