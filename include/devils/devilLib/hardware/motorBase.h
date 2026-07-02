#pragma once
#include "hardwareBase.hpp"

namespace devils
{
    /**
     * Represents some kind of motor or motor group.
     */
    struct IMotor
    {
        virtual ~IMotor() = default;

        /**
         * Controls the motor speed based on the voltage.
         * @param speed The speed to run the motor at, from -1 to 1.
         */
        virtual void move(float speed) = 0;

        /**
         * Stops the motor.
         */
        virtual void stop() = 0;

        /**
         * Gets the current position of the motor in encoder ticks.
         * @return The current position of the motor in encoder ticks.
         */
        virtual HWResult<float> getPosition() = 0;
    };
}
