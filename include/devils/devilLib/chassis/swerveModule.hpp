#pragma once

#include "../hardware/smartMotorGroup.hpp"
#include "../hardware/adiAnalogInput.hpp"
#include "../controller/pidController.hpp"
#include "swerveConstants.hpp"

#include <math.h>

namespace devils
{
    /**
     * Represents a single swerve module, which consists of two motors that can rotate independently to allow for omnidirectional movement and strafing.
     * This module is used in the SwerveChassis to control the movement of the robot.
     */
    struct SwerveModule
    {
    private:
        // MotorA is top, MotorB is bottom
        SmartMotorGroup& motorA;
        SmartMotorGroup& motorB;
        ADIAnalogInput& encoder;

        PIDController angleController = PIDController(0.9f, 0.0f, 0.0f); // TODO: Tune these values

        SwerveConstants swerveConstants;
        int moduleIndex;

        float storedTargetAngle;
        float storedError;

    public:
        SwerveModule(SmartMotorGroup& motorA, SmartMotorGroup& motorB, ADIAnalogInput& encoder, SwerveConstants swerveConstants, int moduleIndex)
            : motorA(motorA), motorB(motorB), encoder(encoder), swerveConstants(swerveConstants), moduleIndex(moduleIndex)
        {
            // Disable brake mode by default to prevent overheating
            motorA.setBrakeMode(false);
            motorB.setBrakeMode(false);
            // Zero encoders
            motorA.setPosition(0);
            motorB.setPosition(0);
        }

        /**
        * Moves the swerve module to the specified angle and drivespeed.
        * @param angle The target angle in radians.
        * @param drivespeed The drivespeed of the wheel from -1 to 1.
        */
        void move(float targetAngle, float drivespeed, bool rotationOptimized = true)
        {
            // Reverse the motors instead of rotating the wheels 180 degrees when the angle is greater than 90 degrees to reduce the time it takes to rotate the wheels
            float passedTargetAngle = targetAngle;
            float alternateTargetAngle = passedTargetAngle + M_PI;
            if (rotationOptimized){
                // Check if the alternate angle is closer to travel to
                if (abs(getAngleError(getPodAngle(), passedTargetAngle)) > abs(getAngleError(getPodAngle(), alternateTargetAngle))){
                    // Set target angle to alternate and reverse drive direction
                    targetAngle = alternateTargetAngle;
                    drivespeed *= -1.0;
                }
                else {
                    // No need to change anything, it'll go to the passed angle
                }
            }
            
            float steeringPower = angleController.update(getAngleError(getPodAngle(), targetAngle));
            // Normalize motor powers so the module rotates well even if we're commanding it to drive at a high speed
            float motorAPower = drivespeed - steeringPower;
            float motorBPower = drivespeed + steeringPower;
            float maxPower = 1.0;
            if(abs(motorAPower) > maxPower) maxPower = abs(motorAPower);
            if (abs(motorBPower) > maxPower) maxPower = abs(motorBPower);

            motorA.move(motorAPower / maxPower);
            motorB.move(motorBPower / maxPower);

            storedTargetAngle = targetAngle;
        }

        float analogToAngle(float input){
            // remap input to 0-2PI, 0 points to the front of the module, clockwise is positive
            // Add calibration offset and make sure the result is less than 2PI
            return fmod(((input * 2*M_PI) - swerveConstants.getCalibrationOffsetAngleByIndex(moduleIndex)), 2*M_PI);
        }

        float getAngleError(float currentAngle, float targetAngle){
            // Normalize any weird target angles over 2PI
            targetAngle = fmod(targetAngle, 2*M_PI);
            float rawAngleDifference = currentAngle - targetAngle;
            float output;
            // Handle weirdness that happens at the discontinuity where 0 meets 2PI
            // Error greater than 180 is impossible, if we see that then we have to go the other way around the circle
            if (abs(rawAngleDifference) > M_PI){
                // Subtract the large slice of the pie from the whole pie to get the small slice, then make sure the sign is right
                output = ((2*M_PI) - abs(rawAngleDifference)) * -(rawAngleDifference / abs(rawAngleDifference));
            } else output = rawAngleDifference;

            storedError = output;
            return output;
        }
        
        float getPodAngle(){
            return analogToAngle(encoder.getValue());
        }

        float getTargetAngle(){
            return storedTargetAngle;
        }

        float getStoredError(){
            return storedError;
        }

        void stop(){
            motorA.move(0);
            motorB.move(0);
        }

    };
}
