#pragma once

#include "pros/rtos.hpp"

namespace devils
{
    class Timer
    {
    public:
        /**
         * Creates a new instance of Timer with a given duration.
         * @param duration The duration of the timer in milliseconds.
         * @return The new instance of Timer.
         */
        Timer(const uint32_t duration) : duration(duration)
        {
        }

        /**
         * Sets the duration of the timer.
         * @param newDuration The duration of the timer in milliseconds.
         */
        void setDuration(const uint32_t newDuration)
        {
            this->duration = newDuration;
        }

        /**
         * Starts the timer.
         */
        void start()
        {
            this->startTime = pros::millis();
            this->isStarted = true;
        }

        /**
         * Stops the timer.
         */
        void stop()
        {
            this->isStarted = false;
        }

        /**
         * Gets whether the timer has started.
         * @return True if the timer has started, false otherwise.
         */
        bool running() const
        {
            return isStarted && !finished();
        }

        /**
         * Gets whether the timer has finished.
         * @return True if the timer has finished, false otherwise.
         */
        bool finished() const
        {
            return isStarted && pros::millis() - startTime >= duration;
        }

        /**
         * Gets the time remaining on the timer.
         * @return The time remaining on the timer in milliseconds.
         */
        uint32_t timeRemaining() const
        {
            if (!running())
                return 0;
            return duration - (pros::millis() - startTime);
        }

    private:
        bool isStarted = false;
        uint32_t startTime = 0;
        uint32_t duration = 0;
    };
}
