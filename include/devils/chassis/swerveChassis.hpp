#pragma once

#include "chassisBase.hpp"
#include "../hardware/smartMotorGroup.hpp"
#include "../hardware/hallEffectEncoder.hpp"
#include "../geometry/vector2.hpp"
#include "../controller/pidController.hpp"

namespace devils
{
    /**
     * Represents a chassis driven by four sets of wheels that can rotate independently, allowing for omnidirectional movement and strafing.
     * This chassis is holonomic, allowing for omnidirectional movement and strafing.
    */
    class SwerveChassis : public ChassisBase
    {
    public:
        /**
         * Creates a new swerve chassis.
         * @param frontLeft The swerve module for the front left wheel.
         * @param frontRight The swerve module for the front right wheel.
         * @param backLeft The swerve module for the back left wheel.
         * @param backRight The swerve module for the back right wheel.
         */
        SwerveChassis(
            SwerveModule& frontLeft,
            SwerveModule& frontRight,
            SwerveModule& backLeft,
            SwerveModule& backRight)
            : frontLeft(frontLeft),
              frontRight(frontRight),
              backLeft(backLeft),
              backRight(backRight)
        {
            
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
        SwerveModule& frontLeft;
        SwerveModule& frontRight;
        SwerveModule& backLeft;
        SwerveModule& backRight;
    };

    /**
     * Represents a single swerve module, which consists of two motors that can rotate independently to allow for omnidirectional movement and strafing.
     * This module is used in the SwerveChassis to control the movement of the robot.
     */
    struct SwerveModule
    {
        SmartMotorGroup& motorA;
        SmartMotorGroup& motorB;
        HallEffectEncoder& encoder;

        /// @brief The offset of the swerve module from the center of the robot in inches
        Vector2 moduleOffset;

        SwerveModule(SmartMotorGroup& motorA, SmartMotorGroup& motorB, HallEffectEncoder& encoder)
            : motorA(motorA), motorB(motorB), encoder(encoder)
        {
            // Disable brake mode by default to prevent overheating
            motorA.setBrakeMode(false);
            motorB.setBrakeMode(false);
        }

        /**
         * Moves the swerve module to the specified angle and magnitude.
         * @param angle The target angle in degrees.
         * @param magnitude The magnitude of the movement from -1 to 1.
         */
        void move(float angle, float magnitude)
        {
            float angleDelta = angleController.update(angle - encoder.getRotation());
            motorA.move(magnitude + angleDelta);
            motorB.move(magnitude - angleDelta);
        }

        private:
            PIDController angleController = PIDController(0.0f, 0.0f, 0.0f); // TODO: Tune these values
    };
}
