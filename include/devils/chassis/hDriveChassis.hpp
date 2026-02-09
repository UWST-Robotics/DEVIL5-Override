#pragma once

#include "chassisBase.hpp"
#include "../hardware/smartMotorGroup.hpp"

namespace devils
{
    /**
     * Represents a chassis driven by the differential of two sets of wheels along with a horizontal set of wheels for strafing.
     * This chassis is holonomic, allowing for omnidirectional movement and strafing.
     */
    class HDriveChassis : public ChassisBase
    {
    public:
        /**
         * Creates a new tank chassis.
         * @param leftMotors The motor group for the left side of the chassis.
         * @param rightMotors The motor group for the right side of the chassis.
         * @param horizontalMotors The motor group for the horizontal wheels of the chassis.
         */
        HDriveChassis(
            SmartMotorGroup& leftMotors,
            SmartMotorGroup& rightMotors,
            SmartMotorGroup& horizontalMotors)
            : leftMotors(leftMotors),
              rightMotors(rightMotors),
              horizontalMotors(horizontalMotors)
        {
            // Disable brake mode by default to prevent overheating
            leftMotors.setBrakeMode(false);
            rightMotors.setBrakeMode(false);
            horizontalMotors.setBrakeMode(false);
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
            moveTank(
                forward + turn,
                forward - turn,
                strafe);
        }

        /**
         * Runs the chassis in voltage mode with individual control of the left and right sides.
         * @param leftSpeed The speed to run the left side of the chassis from -1 to 1.
         * @param rightSpeed The speed to run the right side of the chassis from -1 to 1.
         * @param horizontalSpeed The speed to run the horizontal wheels of the chassis from -1 to 1.
         */
        void moveTank(
            const float leftSpeed,
            const float rightSpeed,
            const float horizontalSpeed = 0) const
        {
            leftMotors.move(leftSpeed);
            rightMotors.move(rightSpeed);
            horizontalMotors.move(horizontalSpeed);
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
         * @return True, since an H-Drive chassis can strafe.
         */
        bool isHolonomic() const override
        {
            return true;
        }

    protected:
        SmartMotorGroup& leftMotors;
        SmartMotorGroup& rightMotors;
        SmartMotorGroup& horizontalMotors;
    };
}
