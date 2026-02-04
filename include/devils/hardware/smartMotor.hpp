#pragma once

#include <algorithm>
#include <string>
#include <cmath>
#include <utility>
#include "pros/motors.hpp"
#include "pros/error.h"
#include "motorBase.h"
#include "hardwareBase.hpp"

namespace devils
{
    /**
     * Represents a single VEX V5 SmartMotor.
     * Use `MotorGroup` to control multiple motors as one.
     */
    class SmartMotor : public IMotor, protected HardwareBase
    {
    public:
        /**
         * Creates a motor object.
         * @param name The name of the motor (for logging purposes)
         * @param port The port of the motor (from 1 to 21)
         */
        SmartMotor(
            std::string name,
            const int8_t port)
            : HardwareBase(std::move(name), "SmartMotor", port),
              motor(port)
        {
        }

        /**
         * Runs the motor in voltage mode.
         * @param speed The speed to run the motor at, from -1 to 1.
         * @return HWStatus indicating success or failure.
         */
        void move(const float speed) override
        {
            // Convert -1 to 1 range to -127 to 127 range
            const auto voltage = std::clamp(static_cast<int>(speed * 127), -127, 127);

            // Pass voltage to motor
            const auto status = motor.move(voltage);
            if (status == PROS_ERR)
                reportError();
        }

        /**
         * Stops the motor.
         * @return HWStatus indicating success or failure.
         */
        void stop() override
        {
            const auto status = motor.brake();
            if (status == PROS_ERR)
                reportError();
        }

        /**
         * Sets the position of the motor in encoder ticks.
         * @param position The position to set the motor to in encoder ticks.
         * @return HWStatus indicating success or failure.
         */
        void setPosition(const float position) const
        {
            const auto status = motor.set_zero_position(position);
            if (status == PROS_ERR)
                reportError();
        }

        /**
         * Sets the brake mode of the motor.
         *
         * \note
         * Brake mode will use the motor's e-brake to stop the motor when `stop()` is called.
         * Coast mode will allow the motor to coast to a stop when `stop()` is called.
         *
         * @param useBrakeMode True to use brake mode, false to coast mode.
         */
        void setBrakeMode(const bool useBrakeMode) const
        {
            const auto prosBrakeMode = useBrakeMode ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST;
            const auto status = motor.set_brake_mode(prosBrakeMode);;
            if (status == PROS_ERR)
                reportError();
        }

        /**
         * Gets the current position of the motor in encoder ticks.
         *
         * \note
         * 1800 ticks/rev with 36:1 gears (red cartridge),
         * 900 ticks/rev with 18:1 gears (green cartridge),
         * 300 ticks/rev with 6:1 gears (blue cartridge)
         *
         * @return The current position of the motor in encoder ticks
         */
        HWResult<float> getPosition() override
        {
            const auto position = static_cast<float>(motor.get_position());
            if (position == PROS_ERR_F)
                return getStatusCode();
            return position;
        }

        /**
         * Returns the current speed of the motor in RPM.
         * @return The current speed of the motor in RPM,
         */
        HWResult<float> getVelocity() const
        {
            const auto velocity = static_cast<float>(motor.get_actual_velocity());
            if (velocity == PROS_ERR_F)
                return getStatusCode();
            return velocity;
        }

        /**
         * Gets the current temperature of the motor in Celsius.
         * @return The current temperature of the motor in Celsius.
         */
        HWResult<float> getTemperature() const
        {
            const auto temperature = static_cast<float>(motor.get_temperature());
            if (temperature == PROS_ERR_F)
                return getStatusCode();
            return temperature;
        }

        /**
         * Gets the current draw of the motor in mA.
         * @return The current draw of the motor in mA.
         */
        HWResult<int> getCurrent() const
        {
            const auto current = motor.get_current_draw();
            if (current == PROS_ERR)
                return getStatusCode();
            return current;
        }

    private:
        pros::Motor motor;
    };
}
