#pragma once

#include "devils/devilLib/devils.h"
#include "./liftSystem.hpp"
#include "./wristSystem.hpp"
#include "./grabberSystem.hpp"
#include <algorithm>

namespace devils{
    class ScoreTask : public AsyncTask
    {
        private:
            WristSystem& wrist;
            LiftSystem& lift;
            GrabberSystem& grabber;

            float neutralDelay = 0.5f;
            float upDelay = 0.75f;
            Timer timer = Timer(0);
            bool isScore = false;
            int step = 0;

        public:
            ScoreTask(
                WristSystem& wrist, LiftSystem& lift, GrabberSystem& grabber)
                :wrist(wrist), lift(lift), grabber(grabber)
        {
        }

        bool setIsScore(bool value)
        {
            isScore = value;
            return isScore;
        }

        bool getIsScore()
        {
            return isScore;
        }

        void onStart() override
        {
            printf("called");
        }

        void onUpdate() override
        {
            printf("ScoreTask updating: %s\n", std::to_string(step).c_str());
            score();
        }

        void score()
        {
            if (isScore == false)
                return;

            if (step == 0)
            {
                wrist.setTarget(0);
                printf("Wrist target position: %f\n", wrist.getTargetPosition());
                timer.setDuration(neutralDelay);
                timer.start();
                step = 1;
            }
            if (step == 1)
            {
                printf("Wrist target position: %f\n", wrist.getTargetPosition());

                if (timer.getIsFinished())
                {
                    step = 2;
                }
            }
            if (step == 2)
            {
                grabber.moveGrabber(-100);
                timer.setDuration(upDelay);
                timer.start();
                step = 3;
            }
            if (step == 3)
            {
                if (timer.getIsFinished())
                {
                    step = 4;
                }
            }
            if (step == 4)
            {
                wrist.setTarget(1);
                lift.moveToPosition(0.0f);
                isScore = false;
                step = 0;
            }
        }

        void passive()
        {
        }
    };
}