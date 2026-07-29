#pragma once

#include "../subsystems/liftSystem.hpp"

namespace devils
{
    class LiftStep : public AutoStep
    {
    public:
        LiftSystem& lift;
        float positionsToMove;
        float maxHeight;
        
        LiftStep(
            LiftSystem& lift,
            const float positionsToMove,
            const float maxHeight)
            : lift(lift),
            positionsToMove(positionsToMove),
            maxHeight(maxHeight)
        {
            lift.setMaxHeight(maxHeight);
        }

        void onUpdate() override
        {
            lift.update();
        }

        void setPositionsToMove(const float positions)
        {
            positionsToMove = positions;
            lift.moveToPosition(lift.convertToPins(lift.getTargetPosition()) + positionsToMove);
        }
    };
}