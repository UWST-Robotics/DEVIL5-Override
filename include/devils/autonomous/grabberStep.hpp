#pragma once

#include "../subsystems/grabberSystem.hpp"

namespace devils
{
    class GrabberStep : public AutoStep
    {
    public:
        GrabberStep(
            GrabberSystem& grabber,
            const float targetSpeed = 1.0)
            : grabber(grabber),
              targetSpeed(targetSpeed)
        {
        }

        void onUpdate() override
        {
            grabber.moveGrabber(targetSpeed);
        }

        /**
         * Sets the target speed of the grabber.
         * @param speed The target speed of the grabber.
         */
        void setTargetSpeed(const float speed)
        {
            targetSpeed = speed;
        }

    private:
        GrabberSystem& grabber;
        float targetSpeed = 1.0;
    };
}