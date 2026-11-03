#pragma once

#include <array>
#include <initializer_list>

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
         * Gets the port numbers for the specified motor name.
         * Replace return values of each case with port numbers.
         * @param name The name of the motor.
         * @return The port numbers for the specified motor name.
         */
        std::initializer_list<int8_t> getPortGroup(Name name)
        {
            std::initializer_list<int8_t> ports;
            currentName = name;
            switch (currentRobot)
            {
                case Robot::BLAZE:
                    switch (currentName)
                    {
                        case Name::grabber:
                            ports = {16};
                            return ports;
                        case Name::intakeTop:
                            ports = {6};
                            return ports;
                        case Name::intakeBottom:
                            ports = {7};
                            return ports;
                        case Name::throughtake:
                            ports = {8};
                            return ports;
                        case Name::wrist:
                            ports = {9};
                            return ports;
                    }
                case Robot::PJ:
                    switch (currentName)
                    {
                        case Name::grabber:
                            ports = {1};
                            return ports;
                        case Name::intakeTop:
                            ports = {2};
                            return ports;
                        case Name::intakeBottom:
                            ports = {3};
                            return ports;
                        case Name::throughtake:
                            ports = {4};
                            return ports;
                        case Name::wrist:
                            ports = {5};
                            return ports;
                    }
                case Robot::ROBIN:
                    switch (currentName)
                    {
                        case Name::grabber:
                            ports = {1};
                            return ports;
                        case Name::intakeTop:
                            ports = {2};
                            return ports;
                        case Name::intakeBottom:
                            ports = {3};
                            return ports;
                        case Name::throughtake:
                            ports = {4};
                            return ports;
                        case Name::wrist:
                            ports = {5};
                            return ports;
                    }
            }
        }

        int getPort(Name name)
        {
            currentName = name;
            switch (currentRobot)
            {
                case Robot::BLAZE:
                    switch (currentName)
                    {
                        case Name::grabber:
                            return 16;
                        case Name::intakeTop:
                            return 6;
                        case Name::intakeBottom:
                            return 7;
                        case Name::throughtake:
                            return 8;
                        case Name::wrist:
                            return 9;
                    }
                case Robot::PJ:
                    switch (currentName)
                    {
                        case Name::grabber:
                            return 1;
                        case Name::intakeTop:
                            return 2;
                        case Name::intakeBottom:
                            return 3;
                        case Name::throughtake:
                            return 4;
                        case Name::wrist:
                            return 5;
                    }
                case Robot::ROBIN:
                    switch (currentName)
                    {
                        case Name::grabber:
                            return 1;
                        case Name::intakeTop:
                            return 2;
                        case Name::intakeBottom:
                            return 3;
                        case Name::throughtake:
                            return 4;
                        case Name::wrist:
                            return 5;
                    }
            }
        }

        private:
        Robot currentRobot{BLAZE};
        Name currentName{grabber};
    };
}