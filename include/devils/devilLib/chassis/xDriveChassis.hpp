#pragma once

#include "chassisBase.hpp"
#include "../hardware/smartMotorGroup.hpp"

namespace devils
{
    /**
     * Represents a chassis driven by a set of 4 omni-wheels at each corner forming an X.
     * This chassis is holonomic, allowing for omnidirectional movement and strafing.
     */
    class XDriveChassis : public ChassisBase
    {
    public:
        /**
         * Creates a new X-Drive chassis.
         * @param frontLeftMotors The motor group for the front left wheel of the chassis.
         * @param frontRightMotors The motor group for the front right wheel of the chassis
         * @param backLeftMotors The motor group for the back left wheel of the chassis
         * @param backRightMotors The motor group for the back right wheel of the chassis
         */
        XDriveChassis(
            SmartMotorGroup& frontLeftMotors,
            SmartMotorGroup& frontRightMotors,
            SmartMotorGroup& backLeftMotors,
            SmartMotorGroup& backRightMotors)
            : frontLeftMotors(frontLeftMotors),
              frontRightMotors(frontRightMotors),
              backLeftMotors(backLeftMotors),
              backRightMotors(backRightMotors)
        {
            // Disable brake mode by default to prevent overheating
            frontLeftMotors.setBrakeMode(false);
            frontRightMotors.setBrakeMode(false);
            backLeftMotors.setBrakeMode(false);
            backRightMotors.setBrakeMode(false);
        }

        /**
         * Runs the chassis in voltage mode.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         * @param strafe The strafe speed of the robot from -1 to 1.
         */
        void move(
            const float forward,
            const float turn,
            const float strafe) override
        {
            const float frontLeftSpeed = forward + turn + strafe;
            const float frontRightSpeed = -forward + turn + strafe;
            const float backLeftSpeed = forward + turn - strafe;
            const float backRightSpeed = -forward + turn - strafe;

            frontLeftMotors.move(frontLeftSpeed);
            frontRightMotors.move(frontRightSpeed);
            backLeftMotors.move(backLeftSpeed);
            backRightMotors.move(backRightSpeed);
        }

        /**
         * Forces the chassis to stop.
         */
        void stop() override
        {
            frontLeftMotors.stop();
            frontRightMotors.stop();
            backLeftMotors.stop();
            backRightMotors.stop();
        }

        /**
         * Checks if the chassis is holonomic, meaning it can strafe in any direction.
         * @return True, since an X-Drive chassis can strafe.
         */
        bool isHolonomic() const override
        {
            return true;
        }

    protected:
        SmartMotorGroup& frontLeftMotors;
        SmartMotorGroup& frontRightMotors;
        SmartMotorGroup& backLeftMotors;
        SmartMotorGroup& backRightMotors;
    };
}
