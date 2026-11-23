#pragma once

#include "../subsystems/wristSystem.hpp"

namespace devils
{
    class WristStep : public AutoStep
    {
    public:
        WristSystem &wrist;
        int desiredPosition;
        float maxHeight;

        WristStep(
            WristSystem &wrist,
            const int desiredPosition)
            : wrist(wrist),
              desiredPosition(desiredPosition)
        {
        }

        void onUpdate() override
        {
            wrist.update();
        }

        /**
         * Sets the position to move the wrist to.
         * @param positions The number position to move to (-1 for down, 0 for neutral, 1 for up).
         */
        void setPosition(const int position)
        {
            wrist.setTarget(position);
        }
    };
}