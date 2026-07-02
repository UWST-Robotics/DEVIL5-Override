#pragma once
#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents a stopwatch that can be used to measure elapsed time.
     * The stopwatch can be started, stopped, and reset.
     * The time can be retrieved in seconds.
     */
    class Stopwatch
    {
    public:
        /**
         * Stops the stopwatch from ticking.
         */
        void stop()
        {
            stopTime = pros::millis();
            isRunning = false;
        }

        /**
         * Resets the stopwatch and starts it from 0.
         */
        void start()
        {
            reset();
            startTime = pros::millis();
            isRunning = true;
        }

        /**
         * Resets the stopwatch without starting it. The time will be reset to 0 and the stopwatch will be stopped.
         */
        void reset()
        {
            startTime = 0;
            stopTime = 0;
            isRunning = false;
        }

        /**
         * Gets the time in seconds since the stopwatch was started.
         * If the stopwatch is stopped, gets the time between when it was started and stopped.
         * @return the time in seconds since the stopwatch was started, or the time between when it was started and stopped if the stopwatch is stopped.
         */
        float getTime() const
        {
            const uint32_t currentTime = isRunning ? pros::millis() : stopTime;
            return (currentTime - startTime) / 1000.0f;
        }

        /**
         * Gets whether the stopwatch is currently running.
         * @return True if the stopwatch is currently running, false otherwise.
         */
        bool getIsRunning() const
        {
            return isRunning;
        }
        
    protected:
        uint32_t startTime = pros::millis();
        uint32_t stopTime = 0;
        bool isRunning = false;
    };
}
