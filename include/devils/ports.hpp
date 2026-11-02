#pragma once

#include <array>
#include <vector>

namespace devils
{
    class Ports
    {
        public:
        /**
         * Represents the name of the motors.
         */
        enum Name
        {
            grabber,
            intakeTop,
            intakeBottom,
            throughtake,
            wrist
        };

        enum Robot
        {
            BLAZE,
            PJ,
            ROBIN
        };

        Ports(Robot robot)
        {
            currentRobot = robot;
        }

        /**
         * Gets the port number for the specified motor name.
         * Replace return values of each case with port numbers.
         * @param name The name of the motor.
         * @return The port number for the specified motor name.
         */
        std::initializer_list<int8_t> getPorts(Name name)
        {
            std::initializer_list<int8_t> ports;
            currentName = name;
            switch (currentRobot)
            {
                case Robot::BLAZE:
                    switch (currentName)
                    {
                        case Name::grabber:
                            ports = {1, 2, 3, 4, 5};
                            return ports;
                            break;
                        case Name ::intakeTop:
                            ports = {6};
                            return ports;
                            break;
                        case Name::intakeBottom:
                            ports = {7};
                            return ports;
                            break;
                        case Name::throughtake:
                            ports = {8};
                            return ports;
                            break;
                        case Name::wrist:
                            ports = {9};
                            return ports;
                            break;
                    }
                case Robot::PJ:
                    switch (currentName)
                    {
                        case Name::grabber:
                            ports = {1};
                            return ports;
                            break;
                        case Name::intakeTop:
                            ports = {2};
                            return ports;
                            break;
                        case Name::intakeBottom:
                            ports = {3};
                            return ports;
                            break;
                        case Name::throughtake:
                            ports = {4};
                            return ports;
                            break;
                        case Name::wrist:
                            ports = {5};
                            return ports;
                            break;
                    }
                case Robot::ROBIN:
                    switch (currentName)
                    {
                        case Name::grabber:
                            ports = {1};
                            return ports;
                            break;
                        case Name::intakeTop:
                            ports = {2};
                            return ports;
                            break;
                        case Name::intakeBottom:
                            ports = {3};
                            return ports;
                            break;
                        case Name::throughtake:
                            ports = {4};
                            return ports;
                            break;
                        case Name::wrist:
                            ports = {5};
                            return ports;
                            break;
                    }
            }
        }

        private:
        Robot currentRobot{BLAZE};
        Name currentName{grabber};
    };
}