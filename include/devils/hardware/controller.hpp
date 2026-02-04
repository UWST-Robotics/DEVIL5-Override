#pragma once

#include "hardwareBase.hpp"
#include "pros/misc.h"

namespace devils
{
    struct ControllerAxis
    {
        struct Options
        {
            /**
             * The minimum joystick input to register movement.
             */
            float deadzone = 0.0f;

            /**
             * The initial value of the joystick curve
             */
            float startingValue = 0.0f;

            /**
             * The ending value of the joystick curve
             */
            float endingValue = 1.0f;

            /**
             * The amount of curve applied to the joystick input.
             */
            float exponent = 1.0f;
        };

        /**
         * Creates a new Controller Joystick Axis.
         * @param port The port of the controller.
         * @param axis The axis of the joystick.
         */
        ControllerAxis(
            const pros::controller_id_e_t port,
            const pros::controller_analog_e_t axis)
            : port(port),
              axis(axis)
        {
        }

        operator float() const
        {
            return getValue();
        }

        /**
         * Gets the raw value of the joystick axis from -1 to 1.
         * @return The raw value of the joystick axis.
         */
        float getRawValue() const
        {
            const auto value = pros::c::controller_get_analog(port, axis);
            return static_cast<float>(value) / 127.0f;
        }

        /**
         * Gets the processed value of the joystick axis from -1 to 1, applying deadzone and curve options.
         * @return The processed value of the joystick axis.
         */
        float getValue() const
        {
            pros::Controller controller(port);
            const float rawValue = static_cast<float>(controller.get_analog(axis)) / 127.0f;

            // Apply deadzone
            if (std::abs(rawValue) < options.deadzone)
                return 0.0f;

            // Apply curve
            const float sign = (rawValue > 0) ? 1.0f : -1.0f;
            const float normalizedValue = (std::abs(rawValue) - options.deadzone) / (1.0f - options.deadzone);
            const float curvedValue = options.startingValue + (options.endingValue - options.startingValue) * std::pow(
                normalizedValue, options.exponent);

            return sign * curvedValue;
        }

        /**
         * Sets the options for the joystick axis.
         * @param newOptions The new options to set.
         */
        void setOptions(const Options& newOptions)
        {
            options = newOptions;
        }

    private:
        Options options;

        pros::controller_id_e_t port;
        pros::controller_analog_e_t axis;
    };

    struct ControllerButton
    {
        enum Mode
        {
            /// @brief True while the button is being pressed.
            PRESSED,

            /// @brief True for one cycle when the button is pressed.
            JUST_PRESSED,

            /// @brief Toggles between true and false each time the button is pressed.
            TOGGLED
        };

        ControllerButton(
            const pros::controller_id_e_t port,
            const pros::controller_digital_e_t button)
            : port(port),
              button(button)
        {
        }

        operator bool()
        {
            return getValue();
        }

        /**
         * Gets the value of the button based on its mode.
         * @return The value of the button.
         */
        bool getValue()
        {
            pros::Controller controller(port);
            switch (mode)
            {
            case PRESSED:
                return controller.get_digital(button);
            case JUST_PRESSED:
                return controller.get_digital_new_press(button);
            case TOGGLED:
                if (controller.get_digital_new_press(button))
                    toggledState = !toggledState;
                return toggledState;
            default:
                return false;
            }
        }

        /**
         * Sets the mode of the button.
         * @param newMode The new mode to set.
         */
        void setMode(const Mode newMode)
        {
            mode = newMode;
            toggledState = false;
        }

    private:
        pros::controller_id_e_t port;
        pros::controller_digital_e_t button;
        Mode mode = PRESSED;
        bool toggledState = false;
    };

    /**
     * Represents a VEX V5 controller.
     */
    struct Controller : HardwareBase
    {
        Controller(const std::string& name, const pros::controller_id_e_t port)
            : HardwareBase(name, "Controller", port),

              a(port, pros::E_CONTROLLER_DIGITAL_A),
              b(port, pros::E_CONTROLLER_DIGITAL_B),
              x(port, pros::E_CONTROLLER_DIGITAL_X),
              y(port, pros::E_CONTROLLER_DIGITAL_Y),

              up(port, pros::E_CONTROLLER_DIGITAL_UP),
              down(port, pros::E_CONTROLLER_DIGITAL_DOWN),
              left(port, pros::E_CONTROLLER_DIGITAL_LEFT),
              right(port, pros::E_CONTROLLER_DIGITAL_RIGHT),

              l1(port, pros::E_CONTROLLER_DIGITAL_L1),
              l2(port, pros::E_CONTROLLER_DIGITAL_L2),
              r1(port, pros::E_CONTROLLER_DIGITAL_R1),
              r2(port, pros::E_CONTROLLER_DIGITAL_R2),

              leftY(port, pros::E_CONTROLLER_ANALOG_LEFT_Y),
              leftX(port, pros::E_CONTROLLER_ANALOG_LEFT_X),
              rightY(port, pros::E_CONTROLLER_ANALOG_RIGHT_Y),
              rightX(port, pros::E_CONTROLLER_ANALOG_RIGHT_X)
        {
        }

        ControllerButton a;
        ControllerButton b;
        ControllerButton x;
        ControllerButton y;

        ControllerButton up;
        ControllerButton down;
        ControllerButton left;
        ControllerButton right;

        ControllerButton l1;
        ControllerButton l2;
        ControllerButton r1;
        ControllerButton r2;

        ControllerAxis leftY;
        ControllerAxis leftX;
        ControllerAxis rightY;
        ControllerAxis rightX;
    };
}
