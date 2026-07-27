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
        // MotorA is top, MotorB is bottom
        SmartMotorGroup& motorA;
        SmartMotorGroup& motorB;
        ADIAnalogInput& encoder;
        float encoderCalibrationOffsetAngle;

        float storedTargetAngle;
        float storedError;

        // Geartrain parameters
        float cartRatio = 6.0;
        int drivePulleyTeeth = 33;
        int drivenPulleyTeeth = 91;
        int sandwichBevelTeeth = 57;
        int wheelBevelTeeth = 25;
        float wheelRadius = (70.0/25.4)/2.0; // 70mm wheel convert to inches

        SwerveModule(SmartMotorGroup& motorA, SmartMotorGroup& motorB, ADIAnalogInput& encoder, float encoderCalibrationOffsetAngle)
            : motorA(motorA), motorB(motorB), encoder(encoder), encoderCalibrationOffsetAngle(encoderCalibrationOffsetAngle)
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
        * @param drivespeed The drivespeed of the movement from -1 to 1.
        */
        void move(float targetAngle, float drivespeed)
        {
            // Reverse the motors instead of rotating the wheels 180 degrees when the angle is greater than 90 degrees to reduce the time it takes to rotate the wheels
            bool enableRotationOptimization = false;
            if (enableRotationOptimization){
                if(abs(getAngleError(getPodAngle(), targetAngle)) > 90){
                    // TODO - this will not work but it's the general idea
                    targetAngle += M_PI;
                    drivespeed *= -1;
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
            return fmod(((input * 2* M_PI) - encoderCalibrationOffsetAngle), 2*M_PI);
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

        float getDistanceDriven(){
            // Uses the motor encoders and geartrain geometry to calculate the total distance a wheel has traveled in inches. Useful for odometry.
            // Vex motors have 50 ticks per rev at the plastic cartridge input gear
            float topBevelRotations = ((motorA.getPosition()/50.0) / cartRatio) * (drivePulleyTeeth / drivenPulleyTeeth);
            float bottomBevelRotations = -((motorB.getPosition()/50.0) / cartRatio) * (drivePulleyTeeth / drivenPulleyTeeth);
            // TODO: Not sure if this next line is correct
            float wheelRotations = (sandwichBevelTeeth / wheelBevelTeeth) * (0.5*(topBevelRotations - bottomBevelRotations));
            return (wheelRotations * 2 * M_PI) * wheelRadius;
        }

        void stop(){
            motorA.move(0);
            motorB.move(0);
        }

        private:
            PIDController angleController = PIDController(0.9f, 0.0f, 0.0f); // TODO: Tune these values
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
         * @param length The length of the robot in inches, measured from center of front swerve pod to center of back swerve pod.
         * @param width The width of the robot in inches, measured from center of left swerve pod to center of right swerve pod.
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
            // This entire function is very ugly right now, ideally we do all this stuff in for loops (and then we can have drivetrains with arbitrary numbers of modules and module locations)
            // Calculate the drive vectors for each wheel based on the forward, turn, and strafe inputs using swerve drive kinematics
            // See https://strykeforce.github.io/classroom/swerve/#/wheel-position-and-speed
            Vector2 translateVect = Vector2(strafe, forward);
            // Each wheel has a different vector that is applied when turning happens
            Vector2 flTurnVect = Vector2(-width/2.0, length/2.0).rotate(-M_PI/2.0).normalize() * turn;
            Vector2 frTurnVect = Vector2(width/2.0, length/2.0).rotate(-M_PI/2.0).normalize() * turn;
            Vector2 blTurnVect = Vector2(-width/2.0, -length/2.0).rotate(-M_PI/2.0).normalize() * turn;
            Vector2 brTurnVect = Vector2(width/2.0, -length/2.0).rotate(-M_PI/2.0).normalize() * turn;

            Vector2 flVect = translateVect + flTurnVect;
            Vector2 frVect = translateVect + frTurnVect;
            Vector2 blVect = translateVect + blTurnVect;
            Vector2 brVect = translateVect + brTurnVect;
            // Calculate the drivespeed and angle for each wheel using the variables calulated above
            float frontLeftSpeed = flVect.magnitude();
            float frontRightSpeed = frVect.magnitude();
            float backLeftSpeed = blVect.magnitude();
            float backRightSpeed = brVect.magnitude();
            // Convert from East-counterclockwise system used by vector2 to North-clockwise system used by swerve modules
            // Yes it would be nice if they used the same angle system but I don't want to break everything
            float angleOffset = M_PI/2.0;
            float frontLeftAngle = -flVect.angleToX() + angleOffset;
            float frontRightAngle = -frVect.angleToX() + angleOffset;
            float backLeftAngle = -blVect.angleToX() + angleOffset;
            float backRightAngle = -brVect.angleToX() + angleOffset;

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
            globalHeading = heading;
            Vector2 inputVector = Vector2(-strafe, forward);
            move(inputVector.rotate(-heading + fieldCentricHeadingOffset).x, turn, inputVector.rotate(-heading + fieldCentricHeadingOffset).y);
        }

        void calibrateFieldCentric(){
            fieldCentricHeadingOffset = -globalHeading;
        }

        // Rotate all wheels to 45deg (X configuration) and stop them. This can be used to prevent the drivetrain from being pushed.
        void plant(){
            frontLeft.move(-M_PI/4.0, 0);
            frontRight.move(M_PI/4.0, 0);
            backLeft.move(M_PI/4.0, 0);
            backRight.move(-M_PI/4.0, 0);
        }

        void home(){
            frontLeft.move(0, 0);
            frontRight.move(0, 0);
            backLeft.move(0, 0);
            backRight.move(0, 0);
        }

        void stop(){
            frontLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
        }

        // Returns an array containing wheel distances traveled and module angles
        void getWheelPositionInfo(){
            
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

        float fieldCentricHeadingOffset;
        float globalHeading;
    };
}
