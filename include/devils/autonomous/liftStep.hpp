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

        /**
         * Sets the number of positions (in pins) to move the lift.
         * @param positions The number of positions (in pins) to move.
         */
        void setPositionsToMove(const float positions)
        {
            positionsToMove = positions;
            lift.moveToPosition(lift.convertToPins(lift.getTargetPosition()) + positionsToMove);
        }
    };
}