#pragma once

#include "chassisBase.hpp"
#include "../hardware/smartMotorGroup.hpp"

namespace devils
{
    /**
     * Represents a chassis driven by the differential of two sets of wheels.
     */
    class TankChassis : public ChassisBase
    {
    public:
        /**
         * Creates a new tank chassis.
         * @param leftMotors The motor group for the left side of the chassis.
         * @param rightMotors The motor group for the right side of the chassis.
         */
        TankChassis(
            SmartMotorGroup& leftMotors,
            SmartMotorGroup& rightMotors)
            : leftMotors(leftMotors),
              rightMotors(rightMotors)
        {
            // Disable brake mode by default to prevent overheating
            leftMotors.setBrakeMode(false);
            rightMotors.setBrakeMode(false);
        }

        /**
         * Runs the chassis in voltage mode.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         * @param strafe The strafe speed of the robot from -1 to 1. (Ignored for tank chassis)
         */
        void move(
            const float forward,
            const float turn,
            const float strafe) override
        {
            moveTank(forward + turn, forward - turn);
        }

        /**
         * Runs the chassis in voltage mode with individual control of the left and right sides.
         * @param leftSpeed The speed to run the left side of the chassis from -1 to 1.
         * @param rightSpeed The speed to run the right side of the chassis from -1 to 1.
         */
        void moveTank(const float leftSpeed, const float rightSpeed) const
        {
            leftMotors.move(leftSpeed);
            rightMotors.move(rightSpeed);
        }

        /**
         * Forces the chassis to stop.
         */
        void stop() override
        {
            leftMotors.stop();
            rightMotors.stop();
        }

        /**
         * Checks if the chassis is holonomic, meaning it can strafe in any direction.
         * @return False, since a tank chassis can't strafe.
         */
        bool isHolonomic() const override
        {
            return true;
        }
        
        /**
         * Gets the motor group for the left side of the chassis.
         * @return The motor group for the left side of the chassis.
         */
        SmartMotorGroup& getLeftMotors() const
        {
            return leftMotors;
        }
        
        /**
         * Gets the motor group for the right side of the chassis.
         * @return The motor group for the right side of the chassis.
         */
        SmartMotorGroup& getRightMotors() const
        {
            return rightMotors;
        }

    protected:
        SmartMotorGroup& leftMotors;
        SmartMotorGroup& rightMotors;
    };
}
