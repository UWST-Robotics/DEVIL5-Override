#pragma once

#include "../subsystems/grabberSystem.hpp"

namespace devils
{
    class GrabberStep : public AutoStep
    {
    public:
        GrabberStep(
            GrabberSystem& grabber)
            : grabber(grabber)
        {
        }

        void onUpdate() override
        {
            grabber.update();
        }

        /**
         * Sets the target speed of the grabber.
         * @param speed The target speed of the grabber.
         */
        void setTargetSpeed(const float speed)
        {
            grabber.moveGrabber(speed);
        }

    private:
        GrabberSystem& grabber;
    };
}