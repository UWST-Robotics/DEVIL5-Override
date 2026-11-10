#pragma once

#include "devils/devilLib/devils.h"
#include "../subsystems/liftSystem.hpp"
#include "../subsystems/clawSystem.hpp"
#include "../subsystems/wristSystem.hpp"
#include "../subsystems/grabberSystem.hpp"
#include "../subsystems/intakeSystem.hpp"
#include "../subsystems/scoreTask.hpp"
namespace devils
{
    class ScuffedRiceLakeAuto
    {
    public:
        static void run(
            SmartMotorGroup fla, SmartMotorGroup flb,
            SmartMotorGroup fra, SmartMotorGroup frb,
            SmartMotorGroup bla, SmartMotorGroup blb,
            SmartMotorGroup bra, SmartMotorGroup brb)
        {
            fla.move(1);
            flb.move(1);

            fra.move(1);
            frb.move(1);

            bla.move(1);
            blb.move(1);

            bra.move(1);
            brb.move(1);

            pros::delay(1500);

            fla.move(0);
            flb.move(0);

            fra.move(0);
            frb.move(0);

            bla.move(0);
            blb.move(0);

            bra.move(0);
            brb.move(0);
            
        }
    };
}
