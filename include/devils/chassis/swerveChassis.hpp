#pragma once

#include "chassisBase.hpp"
#include "../hardware/smartMotorGroup.hpp"
#include "../hardware/adiAnalogInput.hpp"
#include "../geometry/vector2.hpp"
#include "../controller/pidController.hpp"

#include <math.h>

namespace devils
{
    /**
     * Represents a single swerve module, which consists of two motors that can rotate independently to allow for omnidirectional movement and strafing.
     * This module is used in the SwerveChassis to control the movement of the robot.
     */
    struct SwerveModule
    {
        SmartMotorGroup& motorA;
        SmartMotorGroup& motorB;
        ADIAnalogInput& encoder;
        float encoderCalibrationOffsetAngle;

        SwerveModule(SmartMotorGroup& motorA, SmartMotorGroup& motorB, ADIAnalogInput& encoder, float encoderCalibrationOffsetAngle)
            : motorA(motorA), motorB(motorB), encoder(encoder), encoderCalibrationOffsetAngle(encoderCalibrationOffsetAngle)
        {
            // Disable brake mode by default to prevent overheating
            motorA.setBrakeMode(false);
            motorB.setBrakeMode(false);
        }

        /**
        * Moves the swerve module to the specified angle and drivespeed.
        * @param angle The target angle in radians.
        * @param drivespeed The drivespeed of the movement from -1 to 1.
        */
        void move(float targetAngle, float drivespeed)
        {
            float currentAngle = analogToAngle(encoder.getValue());
            std::cout << "current angle: " << currentAngle;
            std::cout << "\n";
            std::cout << "target angle: " << targetAngle;
            std::cout << "\n";
            // Reverse the motors instead of rotating the wheels 180 degrees when the angle is greater than 90 degrees to reduce the time it takes to rotate the wheels
            bool enableRotationOptimization = false;
            if (enableRotationOptimization){
                if(abs(getAngleError(currentAngle, targetAngle)) > 90){
                    // TODO - this will not work but it's the general idea
                    targetAngle += M_PI;
                    drivespeed *= -1;
                }
            }
            
            float steeringPower = angleController.update(getAngleError(currentAngle, targetAngle));
            // Normalize motor powers so the module rotates well even if we're commanding it to drive at a high speed
            float motorAPower = drivespeed + steeringPower;
            float motorBPower = drivespeed - steeringPower;
            float maxPower = 1.0;
            if(abs(motorAPower) > maxPower) maxPower = abs(motorAPower);
            if (abs(motorBPower) > maxPower) maxPower = abs(motorBPower);

            motorA.move(motorAPower / maxPower);
            motorB.move(motorBPower / maxPower);
        }

        float analogToAngle(float input){
            // remap input to 0-2PI, 0 points to the front of the module, clockwise is positive
            // Add calibration offset and make sure the result is less than 2PI
            return fmod(((input * 2* M_PI) + encoderCalibrationOffsetAngle), 2*M_PI);
        }

        float getAngleError(float currentAngle, float targetAngle){
            // Normalize any weird target angles over 2PI
            targetAngle  = fmod(targetAngle, 2*M_PI);
            
            float rawAngleDifference = currentAngle - targetAngle;
            // Handle weirdness that happens at the discontinuity where 0 meets 2PI
            // Error greater than 180 is impossible, if we see that then we have to go the other way around the circle
            if (abs(rawAngleDifference) > M_PI){
                // Subtract the large slice of the pie from the whole pie to get the small slice, then make sure the sign is right
                return ((2*M_PI) - abs(rawAngleDifference)) * -(rawAngleDifference / abs(rawAngleDifference));
            } else return rawAngleDifference;
        }

        private:
            PIDController angleController = PIDController(1.0f, 0.0f, 0.0f); // TODO: Tune these values
    };
        
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
         * @param forward The forward drivespeed of the robot from -1 to 1.
         * @param turn The turn drivespeed of the robot from -1 to 1.
         * @param strafe The strafe drivespeed of the robot from -1 to 1.
         */
        void move(
            const float forward,
            const float turn,
            const float strafe) override
        {
            // Calculate the drivespeed and angle for each wheel based on the forward, turn, and strafe inputs using swerve drive kinematics
            float A = strafe - turn * (length / R);
            float B = strafe + turn * (length / R);
            float C = strafe - turn * (width / R);
            float D = strafe + turn * (width / R);

            // Calculate the drivespeed and angle for each wheel using the variables calulated above
            float frontLeftSpeed = std::sqrt((B * B) + (D * D));
            float frontRightSpeed = std::sqrt((B * B) + (C * C));
            float backLeftSpeed = std::sqrt((A * A) + (D * D));
            float backRightSpeed = std::sqrt((A * A) + (C * C));

            float frontLeftAngle = std::atan2(B, D);
            float frontRightAngle = std::atan2(B, C);
            float backLeftAngle = std::atan2(A, D);
            float backRightAngle = std::atan2(A, C);

            // Normalize the wheel speeds if any of them are greater than 1
            // Yes this is what arrays are for but maybe we'll make it pretty later (almost certainly a lie, never gonna have time for that)
            float maxSpeed = 1.0;
            // max max max max ok yeah an array would be nice
            maxSpeed = std::max(maxSpeed, std::max(std::max(frontRightSpeed, backRightSpeed), std::max(frontLeftSpeed, backLeftSpeed)));
            frontLeftSpeed /= maxSpeed;
            frontRightSpeed /= maxSpeed;
            backLeftSpeed /= maxSpeed;
            backRightSpeed /= maxSpeed;

            // Move each wheel to the calculated angle and drivespeed
            frontLeft.move(frontLeftAngle, frontLeftSpeed);
            frontRight.move(frontRightAngle, frontRightSpeed);
            backLeft.move(backLeftAngle, backLeftSpeed);
            backRight.move(backRightAngle, backRightSpeed);
        }

        // Field-oriented control using the IMU to adjust the angles based on the robot's heading
        void moveFieldCentric(
            const float forward,
            const float turn,
            const float strafe,
            const float heading)
        {
            // Rotate translation vector by negative heading
            Vector2 inputVector = Vector2(strafe, forward);
            inputVector.rotate(-heading);
            move(inputVector.y, turn, inputVector.x);
        }

        /**
         * Checks if the chassis is holonomic, meaning it can strafe in any direction.
         * @return True, since a swerve chassis can strafe, that's kind of the point of all that mechanical design effort.
         */
        bool isHolonomic() const override
        {
            return true;
        }

        void printDebug(){
            //std:cout << 
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
}
