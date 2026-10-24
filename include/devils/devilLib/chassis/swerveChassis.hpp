#pragma once

#include "chassisBase.hpp"
#include "../geometry/vector2.hpp"
#include "swerveConstants.hpp"
#include "swerveModule.hpp"

#include <math.h>

namespace devils
{
    /**
     * Represents a chassis driven by four sets of wheels that can rotate independently, allowing for omnidirectional movement and strafing.
     * This chassis is holonomic, allowing for omnidirectional movement and strafing.
    */
    class SwerveChassis : public ChassisBase
    {
    public:
        float fieldCentricHeadingOffset;
        float constantFCHeadingOffset = M_PI/2.0;
        float globalHeading;
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
            SwerveModule& backRight,
            SwerveConstants swerveConstants)
            : frontLeft(frontLeft),
              frontRight(frontRight),
              backLeft(backLeft),
              backRight(backRight),
              swerveConstants(swerveConstants)
        {

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

            Vector2 flVect = translateVect + swerveConstants.flTurnVect * turn;
            Vector2 frVect = translateVect + swerveConstants.frTurnVect * turn;;
            Vector2 blVect = translateVect + swerveConstants.blTurnVect * turn;;
            Vector2 brVect = translateVect + swerveConstants.brTurnVect * turn;;
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
            move(inputVector.rotate(-(heading + fieldCentricHeadingOffset + constantFCHeadingOffset)).x, turn, inputVector.rotate(-(heading + fieldCentricHeadingOffset + constantFCHeadingOffset)).y);
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
            frontLeft.move(0, 0, false);
            frontRight.move(0, 0, false);
            backLeft.move(0, 0, false);
            backRight.move(0, 0, false);
        }

        void stop() override {
            frontLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
        }

        // TODO:
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

        SwerveConstants swerveConstants;
    };
}
