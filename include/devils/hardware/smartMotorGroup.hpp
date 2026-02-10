#pragma once

#include <string>
#include <utility>
#include "motorBase.h"
#include "smartMotor.hpp"

namespace devils
{
    /**
     * Represents a set of smart motors grouped together.
     */
    class SmartMotorGroup : public IMotor
    {
    public:
        /**
         * Creates a new smart motor group.
         * @param name The name of the motor group (for logging purposes)
         * @param ports The ports of the motors in the group (from 1 to 21)
         */
        SmartMotorGroup(
            std::string name,
            const std::initializer_list<int8_t> ports)
        {
            motors.reserve(ports.size());
            for (int8_t port : ports)
                motors.push_back(std::make_shared<SmartMotor>(getMotorName(port), port));
        }

        /**
         * Runs all the motors in voltage mode.
         * @param voltage The voltage to run the motors at, from -1 to 1.
         */
        void move(const float voltage) override
        {
            for (const auto& motor : motors)
                motor->move(voltage);
        }

        /**
         * Stops all the motors.
         */
        void stop() override
        {
            for (const auto& motor : motors)
                motor->stop();
        }

        /**
         * Sets the position of all the motors in encoder ticks.
         * @param position The position to set all the motors to in encoder ticks.
         */
        void setPosition(const float position) const
        {
            for (const auto& motor : motors)
                motor->setPosition(position);
        }

        /**
         * Gets the motors in the motor group.
         * @return The motors in the motor group.
         */
        std::vector<std::shared_ptr<SmartMotor>>& getMotors()
        {
            return motors;
        }

        /**
         * Sets the brake mode of all the motors in the group.
         * @param useBrakeMode True to use brake mode, false to use coast mode.
         * @return HWStatus indicating success or failure.
         */
        void setBrakeMode(const bool useBrakeMode) const
        {
            for (const auto& motor : motors)
                motor->setBrakeMode(useBrakeMode);
        }

        /**
         * Gets the average position of all the motors in encoder ticks.
         * @return The average position of all the motors in encoder ticks.
         */
        HWResult<float> getPosition() override
        {
            // Iterate through motors and get average position
            int motorCount = 0;
            float position = 0;
            for (const auto& motor : motors)
            {
                const auto motorPosition = motor->getPosition();

                // Skip motors that fail to return position
                if (!motorPosition.isSuccess())
                    continue;

                position += motorPosition;
                motorCount++;
            }

            // Log if no motors returned position
            if (motorCount == 0)
                return ERROR_NO_SUCCESSFUL_RESULTS;

            // Return the mean position
            return position / motorCount;
        }

        /**
         * Returns the average speed of all the motors in RPM.
         * If 1 or more motors fail to return velocity, they are ignored.
         * @return The average speed of all the motors in RPM.
         */
        HWResult<float> getVelocity() const
        {
            // Iterate through motors and get average speed
            int motorCount = 0;
            float speed = 0;
            for (const auto& motor : motors)
            {
                const auto motorSpeed = motor->getVelocity();

                // Skip motors that fail to return velocity
                if (!motorSpeed.isSuccess())
                    continue;

                speed += motorSpeed;
                motorCount++;
            }

            // Log if no motors returned velocity
            if (motorCount == 0)
                return ERROR_NO_SUCCESSFUL_RESULTS;

            // Return the mean speed
            return speed / motorCount;
        }

        /**
         * Returns the average current of all the motors in mA.
         * If 1 or more motors fail to return current, they are ignored.
         * @return The average current of all the motors in mA.
         */
        HWResult<float> getCurrent() const
        {
            // Iterate through motors and get average current
            int motorCount = 0;
            float current = 0;
            for (const auto& motor : motors)
            {
                const auto motorCurrent = motor->getCurrent();

                // Skip motors that fail to return current
                if (!motorCurrent.isSuccess())
                    continue;

                current += motorCurrent;
                motorCount++;
            }

            // Log if no motors returned current
            if (motorCount == 0)
                return ERROR_NO_SUCCESSFUL_RESULTS;

            // Return the mean current
            return current / motorCount;
        }

        /**
         * Returns the average temperature of all the motors in Celsius.
         * @return The average temperature of all the motors in Celsius.
         */
        HWResult<float> getTemperature() const
        {
            // Iterate through motors and get average temperature
            int motorCount = 0;
            float temperature = 0;
            for (const auto& motor : motors)
            {
                const auto motorTemperature = motor->getTemperature();

                // Skip motors that fail to return temperature
                if (!motorTemperature.isSuccess())
                    continue;

                temperature += motorTemperature;
                motorCount++;
            }

            // Log if no motors returned temperature
            if (motorCount == 0)
                return ERROR_NO_SUCCESSFUL_RESULTS;

            // Return the mean temperature
            return temperature / motorCount;
        }

    protected:
        /**
         * Gets the motor name for a specific port.
         * @param port The port of the motor
         * @return The generated motor name
         */
        std::string getMotorName(const int32_t port) const
        {
            return name + "_" + std::to_string(abs(port));
        }

    private:
        const std::string name;
        std::vector<std::shared_ptr<SmartMotor>> motors;
    };
}
