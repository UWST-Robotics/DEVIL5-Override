#pragma once
#include "differentialWheelOdom.hpp"
#include "../utils/backgroundService.hpp"
#include "../hardware/gyroBase.h"

#define M_PI 3.14159265358979323846

namespace devils
{
    /**
     * Represents an odometry system using a tank chassis.
     */
    class TankChassisOdom :
        BackgroundService,
        public DifferentialWheelOdom
    {
    public:
        /**
         * Creates a new tank wheel odometry system.
         * Position is calculated using the left and right encoder values.
         * @param chassis The chassis to use for odometry.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        TankChassisOdom(TankChassis& chassis,
                        const float wheelRadius,
                        const float wheelBase)
            : DifferentialWheelOdom(wheelRadius, wheelBase),
              chassis(chassis)
        {
        }

        /**
         * Sets the number of encoder ticks per full revolution of the wheels.
         * @param newTicksPerRevolution The new ticks per revolution.
         */
        void setTicksPerRevolution(const float newTicksPerRevolution)
        {
            this->ticksPerRevolution = newTicksPerRevolution;
        }

        /**
         * Sets the IMU to use for odometry.
         * @param newIMU The new IMU to use.
         */
        void useIMU(IGyro* newIMU)
        {
            this->imu = newIMU;
        }

    protected:
        void onUpdate() override
        {
            // Process differential odometry
            const float leftPosition = chassis.getLeftMotors().getPosition() / ticksPerRevolution;
            const float rightPosition = chassis.getRightMotors().getPosition() / ticksPerRevolution;
            update(leftPosition, rightPosition);

            // Apply IMU
            if (imu != nullptr)
            {
                Pose currentPose = getPose();
                currentPose.rotation = imu->getHeading();
                setPose(currentPose);
            }
        }

    private:
        float ticksPerRevolution = 300.0 * (48.0 / 36.0); // ticks
        TankChassis& chassis;
        IGyro* imu = nullptr;
    };
}
