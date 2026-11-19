#pragma once

#include "../geometry/vector2.hpp"
#include "devils/devilLib/geometry/math.hpp"

#include <math.h>
#include <numbers>

namespace devils
{
    /**
     * Represents geometric and calibration parameters for swerve drivetrains
     */
    class SwerveConstants
    {
    private:
        // Geartrain parameters, same for all of our swerves
        float bareTicksPerRev = 50.0f;
        float cartRatio = 6.0;
        int drivePulleyTeeth = 33.0f;
        int drivenPulleyTeeth = 91.0f;
        int sandwichBevelTeeth = 57.0f;
        int wheelBevelTeeth = 25.0f;
        float wheelDia = 70.0f / 25.4f; // 70mm wheel convert to inches
    
    public:
        // Used for odom, how far does the bot drive when the top and bottom bevels are moved in opposite directions
        // by one encoder tick total (bottom pos - top pos = 1 tick)
        float linearInchesPerTickDelta = ((drivePulleyTeeth * sandwichBevelTeeth * wheelDia * M_PIF) / (bareTicksPerRev * cartRatio * drivenPulleyTeeth * wheelBevelTeeth)) / 2.0f;

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

        Vector2 flTurnVect2Pod = Vector2(-width/2.0f, 0.0f).normalize();
        Vector2 frTurnVect2Pod = Vector2(width/2.0f, 0.0f).normalize();

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
                    flCalibrationOffsetAngle = 0.0f;
                    frCalibrationOffsetAngle = 0.0f;
                    blCalibrationOffsetAngle = 0.0f;
                    brCalibrationOffsetAngle = 0.0f;
                }
                case 1:{
                    flCalibrationOffsetAngle = -1.0f;
                    frCalibrationOffsetAngle = -3.0f;
                    blCalibrationOffsetAngle = 7.0f;
                    brCalibrationOffsetAngle = 5.0f;
                }
                default:{
                    flCalibrationOffsetAngle = 0.0f;
                    frCalibrationOffsetAngle = 0.0f;
                    blCalibrationOffsetAngle = 0.0f;
                    brCalibrationOffsetAngle = 0.0f;
                }
           }
        }

    };
}
