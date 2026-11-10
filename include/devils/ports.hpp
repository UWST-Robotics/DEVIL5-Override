#pragma once

#include <initializer_list>
#include <cstdint>

namespace devils
{
    /**
    * Tool for managing the port numbers for each robot's motors
    * The port numbers are organized by robot and by motor name.
    * This class allows you to easily retrieve the port numbers for a specific motor on a specific robot.
    * Example usage: SmartMotorGroup intakeMotors = SmartMotorGroup("intake", ports.getPortGroup(Ports::intake));
    * In this example, "intake" is the name of the motor group, and ports.getPortGroup(Ports::intake) retrieves the port numbers for that motor group on the current robot.
    * 'intake' must be defined in the Ports::Name enum, and the port numbers for that motor group must be defined in the portGroups array.
    * The current robot is set when the Ports object is created, and can be changed by creating a new Ports object with a different robot.
    * Example usage: Ports ports = Ports(Ports::Robot::BLAZE);
    */
    class Ports
    {
        public:
        // add new motorgroups here (before COUNT)
        enum Name
        {
            grabber,
            intakeTop,
            intakeBottom,
            throughtake,
            wrist,
            COUNT
        };

        // add new robots here (before ROBOT_COUNT)
        enum Robot
        {
            BLAZE,
            PJ,
            ROBIN,
            ROBOT_COUNT
        };

        Ports(Robot robot) : currentRobot(robot) {}

        /**
         * Returns the port numbers for the specified motor/pneumatic name for the current robot.
         * @param name The name of the motor/pneumatic.
         * @return A list of port numbers for the specified motor/pneumatic.
         */
        std::initializer_list<int8_t> getPortGroup(Name name)
        {
            // returns nothing if the name is invalid
            if (name < 0 || name >= COUNT)
                return {};
            
            // returns nothing if the current robot is invalid
            if (currentRobot < 0 || currentRobot >= ROBOT_COUNT)
                return {};

            // otherwise, return the port group for the current robot and name
            return portGroups[currentRobot][name];
        }

    private:

        Robot currentRobot;

        static const std::initializer_list<int8_t> portGroups[ROBOT_COUNT][COUNT];
    };

    /**
     * Contains the port numbers for each robot's motors.
     * The port numbers are organized by robot and by motor name.
     */
    const std::initializer_list<int8_t> Ports::portGroups[ROBOT_COUNT][COUNT] =
    {
        // BLAZE
        {
            {16}, // grabber
            {6, 7}, // intakeTop
            {8}, // intakeBottom
            {9}, // throughtake
            {10}, // wrist
        },

        // PJ
        {
            {1}, // grabber
            {2}, // intakeTop
            {3}, // intakeBottom
            {4}, // throughtake
            {5}  // wrist
        },

        // ROBIN
        {
            {1}, // grabber
            {2}, // intakeTop
            {3}, // intakeBottom
            {4}, // throughtake
            {5}  // wrist
        }
    };
}