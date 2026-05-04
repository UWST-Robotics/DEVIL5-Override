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
         * @param length The length of the robot in inches.
         * @param width The width of the robot in inches.
         */
        SwerveChassis(
            SwerveModule& frontLeft,
            SwerveModule& frontRight,
            SwerveModule& backLeft,
            SwerveModule& backRight, float length, float width)
            : frontLeft(frontLeft),
              frontRight(frontRight),
              backLeft(backLeft),
              backRight(backRight),
              length(length),
              width(width)
        {
            // Calculate the distance from the center of the robot to each wheel using the Pythagorean theorem
            R = std::sqrt((length * length) + (width * width));
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
            // Calculate the speed and angle for each wheel based on the forward, turn, and strafe inputs using swerve drive kinematics
            float A = strafe - turn * (length / R);
            float B = strafe + turn * (length / R);
            float C = strafe - turn * (width / R);
            float D = strafe + turn * (width / R);

            // Calculate the speed and angle for each wheel using the variables calulated above
            float frontLeftSpeed = std::sqrt((B * B) + (D * D));
            float frontRightSpeed = std::sqrt((B * B) + (C * C));
            float backLeftSpeed = std::sqrt((A * A) + (D * D));
            float backRightSpeed = std::sqrt((A * A) + (C * C));

            float frontLeftAngle = std::atan2(B, D) * 180 / M_PI;
            float frontRightAngle = std::atan2(B, C) * 180 / M_PI;
            float backLeftAngle = std::atan2(A, D) * 180 / M_PI;
            float backRightAngle = std::atan2(A, C) * 180 / M_PI;

            // TODO - Normalize the wheel speeds if any of them are greater than 1
            // TODO - Add field-oriented control using the IMU to adjust the angles based on the robot's heading
            // TODO - Reverse the motors instead of rotating the wheels 180 degrees when the angle is greater than 90 degrees to reduce the time it takes to rotate the wheels

            // Move each wheel to the calculated angle and speed
            frontLeft.move(frontLeftAngle, frontLeftSpeed);
            frontRight.move(frontRightAngle, frontRightSpeed);
            backLeft.move(backLeftAngle, backLeftSpeed);
            backRight.move(backRightAngle, backRightSpeed);
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
        float length;
        float width;
        float R;
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
