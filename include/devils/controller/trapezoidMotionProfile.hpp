#pragma once

#include "../utils/logger.hpp"

namespace devils
{
    class TrapezoidMotionProfile
    {
    public:
        /// @brief A list of constraints for the motion profile.
        struct Constraints
        {
            /// @brief The maximum velocity of the motion profile (typically inches/s or degrees/s).
            float maxVelocity;
            
            /// @brief The maximum acceleration of the motion profile (typically inches/s^2 or degrees/s^2).
            float maxAcceleration;
            
            /// @brief The maximum deceleration of the motion profile (typically inches/s^2 or degrees/s^2).
            float maxDeceleration;
        };
        
        /// @brief An enum representing the current phase of the motion profile.
        enum Phase
        {
            ACCELERATING,
            CRUISING,
            DECELERATING,
            COMPLETE
        };
        
        /// @brief A struct representing the state of the motion profile at a given time.
        struct State
        {
            float time;
            float position;
            float velocity;
            float acceleration;
        };

        /**
         * Calculates a trapezoidal motion profile based on the given constraints, starting velocity, ending velocity, and distance.
         * @param constraints - The constraints for the motion profile, including max velocity, acceleration, and deceleration.
         * @param goalDistance - The total distance to be covered by the motion profile (typically inches or degrees).
         * @param startingVelocity - The initial velocity of the motion profile (typically inches/s or degrees/s).
         * @param endingVelocity - The final velocity of the motion profile (typically inches/s or degrees/s).
         */
        TrapezoidMotionProfile(
            const Constraints& constraints,
            const float goalDistance,
            const float startingVelocity = 0,
            const float endingVelocity = 0)
            : startingVelocity(startingVelocity),
              endingVelocity(endingVelocity),
              constraints(constraints)
        {
            recalculate(goalDistance);
        }

        /**
         * Recalculates the motion profile based on the given distance.
         * Should be called whenever the distance changes to update the motion profile.
         * @param goalDistance - The total distance to be covered by the motion profile (typically inches or degrees).
         */
        void recalculate(const float goalDistance)
        {
            // Check if constraints are zero
            if (constraints.maxVelocity <= 0 ||
                constraints.maxAcceleration <= 0 ||
                constraints.maxDeceleration <= 0)
            {
                Logger::warn("TrapezoidMotionProfile: One or more constraints are zero or negative. Adjusting to default values.");
                constraints.maxVelocity = std::max(constraints.maxVelocity, 1.0f);
                constraints.maxAcceleration = std::max(constraints.maxAcceleration, 1.0f);
                constraints.maxDeceleration = std::max(constraints.maxDeceleration, 1.0f);
            }
            
            // Calculate the time to accelerate/decelerate to/from the peak velocity
            peakVelocity = constraints.maxVelocity;
            accelerationTime = (peakVelocity - startingVelocity) / constraints.maxAcceleration;
            decelerationTime = (peakVelocity - endingVelocity) / constraints.maxDeceleration;

            accelerationDistance = calculateDistance(0, startingVelocity, constraints.maxAcceleration, accelerationTime);
            decelerationDistance = calculateDistance(0, peakVelocity, -constraints.maxDeceleration, decelerationTime);
            
            // Calculate the distance covered during the cruising velocity phase
            const auto positiveGoalDistance = std::abs(goalDistance);
            cruisingDistance = positiveGoalDistance - accelerationDistance - decelerationDistance;
            cruisingTime = cruisingDistance / peakVelocity;
            
            // Check if this is a triangular profile 
            if (cruisingDistance < 0)
            {
                // Recalculate the peak velocity for a triangular profile
                peakVelocity = 2 * positiveGoalDistance +
                                startingVelocity * startingVelocity / constraints.maxAcceleration +
                                endingVelocity * endingVelocity / constraints.maxDeceleration;
                peakVelocity /= 1 / constraints.maxAcceleration + 1 / constraints.maxDeceleration;
                peakVelocity = std::sqrt(peakVelocity);
                
                // Recalculate the times for the triangular profile
                accelerationTime = (peakVelocity - startingVelocity) / constraints.maxAcceleration;
                decelerationTime = (peakVelocity - endingVelocity) / constraints.maxDeceleration;
                
                accelerationDistance = calculateDistance(0, startingVelocity, constraints.maxAcceleration, accelerationTime);
                decelerationDistance = calculateDistance(0, peakVelocity, -constraints.maxDeceleration, decelerationTime);
                
                cruisingTime = 0;
                cruisingDistance = 0;
            }
            
            // Safety check if the calculated times are negative (which can happen if the starting/ending velocities are too high)
            if (accelerationTime < 0)
            {
                Logger::warn("TrapezoidMotionProfile: Negative acceleration time calculated. Adjusting to 0.");
                accelerationTime = 0;
                peakVelocity = startingVelocity;
            }
            if (decelerationTime < 0)
            {
                Logger::warn("TrapezoidMotionProfile: Negative deceleration time calculated. Adjusting to 0.");
                decelerationTime = 0;
                peakVelocity = endingVelocity;
            }
            
            // Copy the sign of the goal distance to the distances and velocities
            const auto goalSign = (goalDistance >= 0) ? 1 : -1;
            accelerationDistance *= goalSign;
            cruisingDistance *= goalSign;
            decelerationDistance *= goalSign;
            peakVelocity *= goalSign;
            acceleration = constraints.maxAcceleration * goalSign;
            deceleration = -constraints.maxDeceleration * goalSign;
        }

        /**
         * Gets the current phase of the motion profile at a given time.
         * @param t - The time at which to get the current phase of the motion profile (typically seconds).
         * @return The current phase of the motion profile (ACCELERATING, CRUISING, DECELERATING, or COMPLETE).
         */
        Phase getPhase(const float t) const
        {
            if (t < accelerationTime)
                return ACCELERATING;
            if (t < accelerationTime + cruisingTime)
                return CRUISING;
            if (t < accelerationTime + cruisingTime + decelerationTime)
                return DECELERATING;
            return COMPLETE;
        }
        
        /**
         * Gets the state of the motion profile at a given time.
         * @param t - The time at which to get the state of the motion profile (typically seconds).
         * @return The state of the motion profile at the given time, including position, velocity, and acceleration.
         */
        State getStateAtTime(const float t) const
        {
            float localT;
            const auto currentPhase = getPhase(t);
            State state{t, 0, 0, 0};
            
            switch (currentPhase)
            {
                case ACCELERATING:
                    state.position = calculateDistance(0, startingVelocity, acceleration, t);
                    state.velocity = startingVelocity + acceleration * t;
                    state.acceleration = acceleration;
                    break;
                case CRUISING:
                    localT = t - accelerationTime;
                    state.position = calculateDistance(accelerationDistance, peakVelocity, 0, localT);
                    state.velocity = peakVelocity;
                    state.acceleration = 0;
                    break;
                case DECELERATING:
                    localT = t - accelerationTime - cruisingTime;
                    state.position = calculateDistance(accelerationDistance + cruisingDistance, peakVelocity, deceleration, localT);
                    state.velocity = peakVelocity + deceleration * localT;
                    state.acceleration = deceleration;
                    break;
                case COMPLETE:
                    state.position = accelerationDistance + cruisingDistance + decelerationDistance;
                    state.velocity = endingVelocity;
                    state.acceleration = 0;
                    break;
            }
            return state;
        }

        /**
         * Gets the total time of the motion profile.
         * @return The total time of the motion profile (typically seconds).
         */
        float getTotalTime() const
        {
            return accelerationTime + cruisingTime + decelerationTime;
        }
        
    protected:
        /**
         * Calculates the distance covered during a segment of motion given an initial velocity, constant acceleration, and time.
         * @param initialPosition - The initial position of this segment (typically inches or degrees).
         * @param initialVelocity - The initial velocity of this segment (typically inches/s or degrees/s).
         * @param acceleration - A constant acceleration of this segment (typically inches/s^2 or degrees/s^2).
         * @param time - The duration of this segment (typically seconds).
         * @return The distance covered during this segment (typically inches or degrees).
         */
        static float calculateDistance(
            const float initialPosition,
            const float initialVelocity,
            const float acceleration,
            const float time)
        {
            return initialPosition + initialVelocity * time + 0.5f * acceleration * time * time;
        }
        
    private:
        float startingVelocity;
        float endingVelocity;
        Constraints constraints;
        
        float accelerationDistance;
        float cruisingDistance;
        float decelerationDistance;
        
        float accelerationTime;
        float cruisingTime;
        float decelerationTime;
        
        float acceleration;
        float deceleration;
        
        float peakVelocity;
    };
}