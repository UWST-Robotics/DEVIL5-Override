#pragma once

#include "devils/devilLib/devils.h"
#include "devils/devilLib/hardware/smartMotor.hpp"
#include <algorithm>

namespace devils{
    //PID controller for the wrist motors
    PIDController wristPid = PIDController(0.0025, 0.0000, 0.0000);

    class WristSystem{
        public:
            WristSystem(
                SmartMotorGroup& wristMotors)
                :wristMotors(wristMotors)
        {
        }

        float target = DOWN;
        int targetState = -1;


        float getPosition(){
            return wristMotors.getPosition();
        }

        float getTargetPosition()
        {
            return target;
        }

        float getTargetState()
        {
            return targetState;
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
            printf("i got called, boooooooooo 1");
            target = DOWN;
            targetState = -1;
            printf("%0.2f", getPosition());
            printf("%0.2f", wristMotors.getPosition());
        }

        /**
         * Moves the wrist to the middle/neutral position
         */
        void moveNeutral(){
            printf("i got called, yayyyyyyyyyy");
            target = NEUTRAL;
            targetState = 0;
            printf("%0.2f", getPosition());
            printf("%0.2f", wristMotors.getPosition());
        }

        /**
         * Moves the wrist all the way up
         */
        void moveUp(){
            printf("i got called, boooooooooo 2");
            target = UP;
            targetState = 1;
            printf("%0.2f", getPosition());
            printf("%0.2f", wristMotors.getPosition());
        }

        /**
         * Moves wrist to a set target
         * Allows for easier driver control of the wrist
         */
        void setTarget(int target)
        {
            if (target > 1)
                target = 1;
            else if (target < -1)
                target = -1;

            if (target == -1)
                moveDown();
            else if (target == 0)
                moveNeutral();
            else if (target == 1)
                moveUp();
        }

    private:
        static constexpr float DOWN = 0;
        static constexpr float NEUTRAL = 750;
        static constexpr float UP = 1300;

        SmartMotorGroup& wristMotors;
    
    };
}