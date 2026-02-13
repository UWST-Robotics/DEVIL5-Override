#pragma once

#include "pros/rtos.hpp"

namespace devils
{
    class Timer
    {
    public:
        /**
         * Creates a new instance of Timer with a given duration.
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
            return isStarted && elapsedTime() >= duration;
        }

        /**
         * Gets the elapsed time since the timer was started.
         * @return The elapsed time since the timer was started in milliseconds.
         */
        float elapsedTime() const
        {
            return static_cast<float>(pros::millis() - startTime) * 1000.0f;
        }
        
        /**
         * Gets the time remaining on the timer.
         * @return The time remaining on the timer in milliseconds.
         */
        float timeRemaining() const
        {
            if (!running())
                return 0;
            return duration - elapsedTime();
        }

    private:
        bool isStarted = false;
        uint32_t startTime = 0;
        float duration = 0;
    };
}
