#pragma once

#include "differentialWheelOdom.hpp"
#include "../utils/asyncTask.hpp"

namespace devils
{
    /**
     * Represents an odometry system using a set of parallel rotation sensors.
     * If the sensors are perpendicular, use `PerpendicularSensorOdometry` instead.
     */
    class ParallelSensorOdometry : public DifferentialWheelOdom, public AsyncTask
    {
    public:
        /**
         * Creates an odometry system using a set of parallel rotation sensors.
         * @param leftSensor The left rotation sensor.
         * @param rightSensor The right rotation sensor.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        ParallelSensorOdometry(RotationSensor& leftSensor,
                               RotationSensor& rightSensor,
                               const float wheelRadius,
                               const float wheelBase)
            : DifferentialWheelOdom(wheelRadius, wheelBase),
              leftSensor(leftSensor),
              rightSensor(rightSensor)
        {
        }

        void onUpdate() override
        {
            // Get sensor angles
            const auto leftResult = leftSensor.getAngle();
            const auto rightResult = rightSensor.getAngle();
            
            if (!leftResult.isSuccess() ||
                !rightResult.isSuccess())
            {
                Logger::warn("Failed to get sensor angles for ParallelSensorOdometry.");
                return;
            }
            
            // Convert angles to wheel positions in revolutions
            const float leftPosition = leftResult.value / (2 * M_PIF) / ticksPerRevolution;
            const float rightPosition = rightResult.value / (2 * M_PIF) / ticksPerRevolution;
            update(leftPosition, rightPosition);
        }

        /**
         * Sets the number of encoder ticks per full revolution of the wheels.
         * @param newTicksPerRevolution The new ticks per revolution.
         */
        void setTicksPerRevolution(const float newTicksPerRevolution)
        {
            this->ticksPerRevolution = newTicksPerRevolution;
        }

    private:
        float ticksPerRevolution = 300.0 * (48.0 / 36.0); // ticks
        RotationSensor& leftSensor;
        RotationSensor& rightSensor;
    };
}
