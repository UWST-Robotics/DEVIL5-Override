#pragma once

#include "trapezoidMotionProfile.hpp"
#include "pidController.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents a controller for `TrapezoidMotionProfile` that allows you to get the current state of the motion profile over a time period.
     * Based on https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
     */
    class ProfiledPIDController
    {
    public:
        /**
         * Makes a controller for a new `TrapezoidMotionProfile` based on the given constraints, distance, starting velocity, and ending velocity.
         * @param pidOptions - The options for the internal PID controller, which can be used as a feedback controller to correct for any errors in the motion profile.
         * @param constraints - The constraints for the motion profile, including max velocity, acceleration, and deceleration.
         * @param goalDistance - The total distance to be covered by the motion profile (typically inches or degrees).
         * @param startingVelocity - The initial velocity of the motion profile (typically inches/s or degrees/s).
         * @param endingVelocity - The final velocity of the motion profile (typically inches/s or degrees/s).
         */
        ProfiledPIDController(
            const PIDController::Options& pidOptions,
            const TrapezoidMotionProfile::Constraints& constraints,
            const float goalDistance,
            const float startingVelocity = 0,
            const float endingVelocity = 0)
            : pidController(pidOptions),
              motionProfile(constraints, goalDistance, startingVelocity, endingVelocity),
              internalTimer(motionProfile.getTotalDuration())
        {
        }

        /**
         * Resets the internal timer to 0 and restarts the motion profile from the beginning
         */
        void reset()
        {
            pidController.reset();
            internalTimer.start();
        }

        /**
         * Calculates the time remaining in the motion profile based on the internal timer.
         * @return The time remaining in the motion profile in seconds. Returns 0 if the motion profile is complete.
         */
        float getTimeRemaining() const
        {
            return internalTimer.getTimeRemaining();
        }

        /**
         * Gets the total duration of the motion profile.
         * @return The total duration of the motion profile in seconds.
         */
        float getTotalDuration() const
        {
            return motionProfile.getTotalDuration();
        }

        /**
         * Gets the setpoint which is the target state of the motion profile based on the internal timer.
         * @param deltaTime - The amount of time to add to the internal timer to account for feedback latency. This can be used to get a more accurate setpoint for the current time when using the `update` method, which may have some latency between getting the setpoint and applying the output.
         * @return The current state of the motion profile based on the internal timer.
         */
        TrapezoidMotionProfile::State getSetpoint(const float deltaTime = 0.0f) const
        {
            const auto t = internalTimer.getElapsedTime() + deltaTime;
            return motionProfile.getStateAtTime(t);
        }

        /**
         * Gets the current error of the motion profile based on the current position and the setpoint. This can be used as feedback to correct for any errors in the motion profile.
         * @param currentPosition - The current position of the system (typically inches or degrees).
         * @return The current error of the motion profile based on the current position and the setpoint. Returns 0 if the motion profile is complete.
         */
        float update(const float currentPosition)
        {
            const auto setpoint = getSetpoint(-feedbackDelay);
            return pidController.update(setpoint.position - currentPosition);
        }

        /**
         * Sets a new goal distance for the motion profile and resets the internal timer.
         * Should be called whenever the goal distance changes to update the motion profile.
         * @param newGoal - The new goal distance to be covered by the motion profile (typically inches or degrees).
         * @param newStartingVelocity - The new initial velocity of the motion profile (typically inches/s or degrees/s). If not provided, the starting velocity will be 0.
         * @param newEndingVelocity - The new final velocity of the motion profile (typically inches/s or degrees/s). If not provided, the ending velocity will be 0.
         */
        void setGoal(const float newGoal,
                     const float newStartingVelocity = 0,
                     const float newEndingVelocity = 0)
        {
            motionProfile.recalculate(newGoal, newStartingVelocity, newEndingVelocity);
            internalTimer.setDuration(motionProfile.getTotalDuration());
            reset();
        }

        /**
         * Adds a delay to the internal timer to account for feedback latency.
         * Only used in the `update` method to get a more accurate setpoint for the current time when there may be some latency between getting the setpoint and applying the output.
         * @param delay - The amount of time to add to the internal timer to account for feedback latency in seconds.
         */
        void setFeedbackDelay(const float delay)
        {
            feedbackDelay = delay;
        }
        
    protected:
        PIDController pidController;
        TrapezoidMotionProfile motionProfile;
        Timer internalTimer;
        float feedbackDelay = 0.02f; // seconds
    };
}
