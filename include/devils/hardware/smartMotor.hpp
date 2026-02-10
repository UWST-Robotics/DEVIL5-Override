#pragma once

#include <algorithm>
#include <string>
#include <cmath>
#include <utility>
#include "pros/motors.hpp"
#include "motorBase.h"
#include "hardwareBase.hpp"

namespace devils
{
    /**
     * Represents a single VEX V5 SmartMotor.
     * Use `MotorGroup` to control multiple motors as one.
     */
    class SmartMotor : public IMotor, V5HardwareBase
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
            : V5HardwareBase(std::move(name), "SmartMotor", abs(port)),
              isInverted(port < 0)
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
            auto voltage = std::clamp(static_cast<int>(speed * 127), -127, 127);

            // Invert voltage if necessary
            if (isInverted)
                voltage = -voltage;

            // Pass voltage to motor
            executeWithErrorCheck<int32_t>(pros::c::motor_move_voltage, port, voltage);
        }

        /**
         * Stops the motor.
         * @return HWStatus indicating success or failure.
         */
        void stop() override
        {
            executeWithErrorCheck<int32_t>(pros::c::motor_brake, port);
        }

        /**
         * Sets the position of the motor in encoder ticks.
         * @param position The position to set the motor to in encoder ticks.
         * @return HWStatus indicating success or failure.
         */
        void setPosition(const float position)
        {
            // Convert position to a double
            const auto positionDouble = static_cast<double>(position);
            
            // Pass position to motor
            executeWithErrorCheck<int32_t>(pros::c::motor_set_zero_position, port, positionDouble);
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
        void setBrakeMode(const bool useBrakeMode)
        {
            // Convert to PROS status
            const auto prosBrakeMode = useBrakeMode ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST;

            // Pass brake mode to motor
            executeWithErrorCheck<int32_t>(pros::c::motor_set_brake_mode, port, prosBrakeMode);
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
            // Get the position from the motor
            const auto result = executeWithErrorCheck<double>(pros::c::motor_get_position, port);
            if (!result.isSuccess())
                return result.status;
            
            // Invert the position if necessary
            auto position = static_cast<float>(result.value);
            if (isInverted)
                position = -position;
            
            return position;
        }

        /**
         * Returns the current speed of the motor in RPM.
         * @return The current speed of the motor in RPM.
         */
        HWResult<float> getVelocity()
        {
            // Get the velocity from the motor
            const auto result = executeWithErrorCheck<double>(pros::c::motor_get_actual_velocity, port);
            if (!result.isSuccess())
                return result.status;
            
            // Invert the velocity if necessary
            auto velocity = static_cast<float>(result.value);
            if (isInverted)
                velocity = -velocity;
            
            return velocity;
        }

        /**
         * Gets the current temperature of the motor in Celsius.
         * @return The current temperature of the motor in Celsius.
         */
        HWResult<float> getTemperature()
        {
            // Get the temperature from the motor
            const auto result = executeWithErrorCheck<double>(pros::c::motor_get_temperature, port);
            if (!result.isSuccess())
                return result.status;
            
            return static_cast<float>(result.value);
        }

        /**
         * Gets the current draw of the motor in amps.
         * @return The current draw of the motor in amps.
         */
        HWResult<float> getCurrent()
        {
            // Get the current from the motor
            const auto result = executeWithErrorCheck<int32_t>(pros::c::motor_get_current_draw, port);
            if (!result.isSuccess())
                return result.status;
            
            // Convert from milliamps to amps
            return static_cast<float>(result.value) / 1000.0f;
        }

    protected:
        bool isInverted = false;
    };
}
