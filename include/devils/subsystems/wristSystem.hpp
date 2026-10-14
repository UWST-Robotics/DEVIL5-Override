#pragma once

#include "devils/devilLib/devils.h"
#include "devils/devilLib/hardware/smartMotor.hpp"
#include <algorithm>

namespace devils{
    //PID controller for the wrist motors
    PIDController wristPid = PIDController(0.01, 0.0000, 0.0000);

    class WristSystem{
        public:
            WristSystem(
                SmartMotorGroup& wristMotors,
                RotationSensor& wristSensor)
                :wristMotors(wristMotors),
                wristSensor(wristSensor)
        {
        }

        float target = DOWN;

        float getPosition(){
            return wristSensor.getAngle();
        }

        float getError(){
            float error;
            error = target-getPosition();
            return error;
        }
        
        void update(){
            wristMotors.move(wristPid.update(getError()));
        }

        /**
         * Moves the wrist all the way down
         */
        void moveDown(){
            target = DOWN;
            printf("%0.2f", getPosition());
            printf("%0.2f", wristMotors.getPosition());
        }

        /**
         * Moves the wrist to the middle/neutral position
         */
        void moveNeutral(){
            target = NEUTRAL;
            printf("%0.2f", getPosition());
            printf("%0.2f", wristMotors.getPosition());
        }

        /**
         * Moves the wrist all the way up
         */
        void moveUp(){
            target = UP;
            printf("%0.2f", getPosition());
            printf("%0.2f", wristMotors.getPosition());
        }

    private:
        static constexpr float DOWN = 0;
        static constexpr float NEUTRAL = -(45);
        static constexpr float UP = -90;

        SmartMotorGroup& wristMotors;
        RotationSensor& wristSensor;
    
    };
}