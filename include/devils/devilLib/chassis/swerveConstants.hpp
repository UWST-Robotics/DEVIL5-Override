#pragma once

#include "../geometry/vector2.hpp"

#include <math.h>

namespace devils
{
    /**
     * Represents geometric and calibration parameters for swerve drivetrains
     */
    class SwerveConstants
    {
    private:
        // Geartrain parameters, same for all of our swerves
        float cartRatio = 6.0;
        int drivePulleyTeeth = 33;
        int drivenPulleyTeeth = 91;
        int sandwichBevelTeeth = 57;
        int wheelBevelTeeth = 25;
        float wheelRadius = (70.0/25.4)/2.0; // 70mm wheel convert to inches
    
    public:
        // Used for odom, how far does the bot drive when the top and bottom bevels are moved in opposite directions
        // by one encoder tick total (bottom pos - top pos = 1 tick)
        float linearInchesPerTickDelta = 0.0;

        // Uses the motor encoders and geartrain geometry to calculate the total distance a wheel has traveled in inches. Useful for odometry.
        // Vex motors have 50 ticks per rev at the plastic cartridge input gear
        //float topBevelRotations = ((motorA.getPosition()/50.0f) / cartRatio) * (drivePulleyTeeth / drivenPulleyTeeth);
        //float bottomBevelRotations = -((motorB.getPosition()/50.0f) / cartRatio) * (drivePulleyTeeth / drivenPulleyTeeth);
        // TODO: Not sure if this next line is correct
        //float wheelRotations = (sandwichBevelTeeth / wheelBevelTeeth) * (0.5f*(topBevelRotations - bottomBevelRotations));
        //return (wheelRotations * 2 * M_PI) * wheelRadius;

        // Drivetrain geometry parameters, same for all of our swerves (for now)
        float length;
        float width;
        float R = std::sqrt((length * length) + (width * width));

        // Each wheel has a different vector that is applied when turning happens
        Vector2 flTurnVect = Vector2(-width/2.0f, length/2.0f).rotate(-M_PI/2.0).normalize();
        Vector2 frTurnVect = Vector2(width/2.0f, length/2.0f).rotate(-M_PI/2.0).normalize();
        Vector2 blTurnVect = Vector2(-width/2.0f, -length/2.0f).rotate(-M_PI/2.0).normalize();
        Vector2 brTurnVect = Vector2(width/2.0f, -length/2.0f).rotate(-M_PI/2.0).normalize();

        // Calibration data, different for every swerve and every module, changes mildly frequently
        float flCalibrationOffsetAngle;
        float frCalibrationOffsetAngle;
        float blCalibrationOffsetAngle;
        float brCalibrationOffsetAngle;
        /*
          ^
          |
        forward
          |
        0   1
              ---right--->
        2   3
        */
        float getCalibrationOffsetAngleByIndex(int index){
            switch (index) {
                case 0: return flCalibrationOffsetAngle;
                case 1: return frCalibrationOffsetAngle;
                case 2: return blCalibrationOffsetAngle;
                default: return brCalibrationOffsetAngle;
            }
        }

        // Constructor
        SwerveConstants(int botID, float length, float width)
            : length(length), width(width)
        {
            // Bot ID is 0 for Blaze, 1 for PJ, 2 for Robin, etc.
            switch (botID) {
                case 0:{
                    flCalibrationOffsetAngle = 8.1f;
                    frCalibrationOffsetAngle = 1.9;
                    blCalibrationOffsetAngle = 4.9f;
                    brCalibrationOffsetAngle = 4.7f;
                }
                case 1:{
                    flCalibrationOffsetAngle = -2.3f;
                    frCalibrationOffsetAngle = -0.3f;
                    blCalibrationOffsetAngle = 3.4f;
                    brCalibrationOffsetAngle = 2.6f;
                }
                default:{
                    flCalibrationOffsetAngle = 0.0;
                    frCalibrationOffsetAngle = 0.0;
                    blCalibrationOffsetAngle = 0.0;
                    brCalibrationOffsetAngle = 0.0;
                }
           }
        }

    };
}
