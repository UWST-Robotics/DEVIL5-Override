#pragma once
#include "../subsystems/StickSystem.hpp"

namespace devils
{
    class StickAutoStep : public AutoStep
    {
    public:
        StickAutoStep(StickSystem& stickSystem)
            : stickSystem(stickSystem)
        {
        }

        void onUpdate() override
        {
            stickSystem.moveStick();
        }

    private:
        StickSystem& stickSystem;
    };
}