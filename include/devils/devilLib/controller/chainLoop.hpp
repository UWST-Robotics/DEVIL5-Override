#pragma once
#include "../hardware/smartMotorGroup.hpp"
#include "../geometry/math.hpp"

namespace devils
{
    /**
     * Represents a conveyor chain loop with a specified length.
     * This class is used to calculate the position of the conveyor chain loop in chain links.
     */
    class ChainLoop
    {
    public:
        /**
         * Represents a conveyor chain loop with a specified length.
         * @param motorGroup The SmartMotorGroup that represents the motors in the chain loop.
         * @param sprocketTeeth The amount of teeth on the sprocket.
         * @param length The amount of chain in the loop.
         * @param startingOffset The offset of the first motor in the loop.
         */
        ChainLoop(SmartMotorGroup& motorGroup,
                  const int sprocketTeeth,
                  const int length,
                  const float startingOffset = 0)
            : motorGroup(motorGroup),
              sprocketTeeth(sprocketTeeth),
              length(length),
              startingOffset(startingOffset)
        {
        }

        /**
         * Gets the position of the loop in chain links.
         */
        float getPosition()
        {
            // Get the encoder position
            const auto encoderPosition = motorGroup.getPosition();
            if (!encoderPosition.isSuccess())
                return lastPosition;

            // Get the revolutions of the sprocket
            const float revolutions = encoderPosition / ENCODER_TICKS_PER_REVOLUTION;

            // Get the final position in chain links
            const float position = revolutions * static_cast<float>(sprocketTeeth) + startingOffset;

            // Modulo the position to keep it within the length of the chain loop
            lastPosition = Math::signedMod(position, static_cast<float>(length));
            return lastPosition;
        }

        /**
         * Gets the distance to a target position in chain links.
         * @param targetPosition The target position in chain links.
         * @return The distance to the target position in chain links.
         */
        float getDistanceToPosition(const float targetPosition)
        {
            // Get the current position of the loop
            const float currentPosition = getPosition();

            // Calculate the distance to the target position
            const float distance = Math::signedMod(targetPosition - currentPosition, static_cast<float>(length));

            // Return the distance to the target position
            return distance;
        }

    private:
        /// @brief The amount of encoder ticks per revolution of the conveyor motors.
        static constexpr float ENCODER_TICKS_PER_REVOLUTION = 300.0;

        SmartMotorGroup& motorGroup;
        int sprocketTeeth = 0;
        int length = 0;
        float startingOffset = 0;

        float lastPosition = 0;
    };
}
