#pragma once

#include "pros/misc.hpp"
#include "../hardware/controller.hpp"

namespace devils
{
    /**
     * Represents a robot control system.
     * Can be swapped out for different robots at compile time
     * by modifying the type defined in `main.cpp` file.
     */
    struct Robot
    {
        virtual ~Robot() = default;

        /**
         * Ran when field control is connected.
         */
        virtual void competition()
        {
        }

        /**
         * Ran when the robot is disabled
         */
        virtual void disabled()
        {
        }

        /**
         * Ran at the start of the Autonomous period.
         */
        virtual void autonomous()
        {
        }

        /**
         * Ran at the start of the Teleoperated period.
         */
        virtual void opcontrol()
        {
        }

        /// @brief The main game controller.
        Controller mainController = Controller("Main Controller", pros::E_CONTROLLER_MASTER);

        /// @brief The partner game controller.
        Controller partnerController = Controller("Partner Controller", pros::E_CONTROLLER_PARTNER);
    };
}
