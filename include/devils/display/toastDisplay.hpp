#pragma once

#include "displayBase.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/extra/widgets/list/lv_list.h"
#include "../utils/backgroundService.hpp"
#include "../utils/logger.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    class ToastDisplay :
        public BackgroundService,
        public DisplayBase
    {
    public:
        ToastDisplay()
        {
            // Load Toast Container
            toastContainer = lv_list_create(getRootContainer());
            lv_obj_set_size(toastContainer, OVERLAY_WIDTH, OVERLAY_HEIGHT);
            lv_obj_center(toastContainer);
            lv_obj_set_style_bg_opa(toastContainer, 0, 0);
            lv_obj_set_style_border_opa(toastContainer, 0, 0);
        }

        ~ToastDisplay() override
        {
            lv_obj_del(toastContainer);
        }

    protected:
        /// @brief Represents a single error popup being displayed on the screen.
        struct ActiveToast
        {
            uint32_t timestamp;
            std::string text;
            lv_obj_t* toastObject;
        };

        /**
         * Creates a toast object in LVGL with the given text and adds it to the toast container.
         * @param logMessage - The log message to create a toast for. The text of the log message will be displayed in the toast.
         * @return The created toast object.
         */
        lv_obj_t* createToast(const Logger::LogMessage& logMessage) const
        {
            // Get proper icon and color for the toast based on the log level of the message
            const auto toastIcon = getIconFromLogLevel(logMessage.level);
            const auto toastColor = getColorFromLogLevel(logMessage.level);

            // Create the toast as a button
            const auto button = lv_list_add_btn(
                toastContainer,
                toastIcon.c_str(),
                logMessage.text.c_str());
            lv_obj_set_style_bg_color(button, toastColor, 0);

            return button;
        }

        void onUpdate() override
        {
            // Log any new log messages as toasts
            while (!Logger::toastBuffer.empty())
            {
                onLogMessage(Logger::toastBuffer.top());
                Logger::toastBuffer.pop();
            }
        }

        /**
         * Called every time a log message is logged. 
         * If the log message is an error, it creates a toast for the error message and adds it to the screen.
         * @param logMessage - The log message to display as a toast if it's an error message.
         */
        void onLogMessage(const Logger::LogMessage& logMessage)
        {
            // Ignore messages that aren't at least the current log level
            if (logMessage.level < logLevel)
                return;

            // Check if the error is already being displayed
            for (auto& toast : activeToasts)
            {
                if (toast.text == logMessage.text)
                {
                    // Update the timestamp to keep it on screen longer
                    toast.timestamp = pros::millis();
                    return;
                }
            }

            // Add the toast to the list of active toasts
            activeToasts.push_back({
                pros::millis(),
                logMessage.text,
                createToast(logMessage)
            });
        }

        /**
         * Sets the log level for the toast display. Only log messages at least this severe will be displayed as toasts.
         * @param newLogLevel - The new log level to set for the toast display. Only log messages at least this severe will be displayed as toasts.
         */
        void setLogLevel(const Logger::LogLevel newLogLevel)
        {
            logLevel = newLogLevel;
        }

        /**
         * Takes the log level of a message and returns the icon to use for a toast based on the log level of the message.
         * @param logLevel - The log level of the message to get the icon for.
         * @return The icon to use for a toast based on the log level of the message. Errors are X's, warnings are !'s, etc.
         */
        static std::string getIconFromLogLevel(const Logger::LogLevel logLevel)
        {
            switch (logLevel)
            {
            case Logger::ERROR:
                return LV_SYMBOL_CLOSE;
            case Logger::WARN:
                return LV_SYMBOL_WARNING;
            case Logger::INFO:
                return LV_SYMBOL_PLUS;
            default:
                return LV_SYMBOL_MINUS;
            }
        }

        /**
         * Gets the color to use for a toast based on the log level of the message.
         * Errors are red, warnings are yellow, etc.
         * @param logLevel - The log level of the message to get the color for.
         * @return The color to use for a toast based on the log level of the message. 
         */
        static lv_color_t getColorFromLogLevel(const Logger::LogLevel logLevel)
        {
            switch (logLevel)
            {
            case Logger::ERROR:
                return lv_color_hex(0xE94E4E);
            case Logger::WARN:
                return lv_color_hex(0xE6BA01);
            case Logger::INFO:
                return lv_color_hex(0x078BEB);
            default:
                return lv_color_hex(0xA8A8A8);
            }
        }

    private:
        static constexpr int OVERLAY_WIDTH = 440;
        static constexpr int OVERLAY_HEIGHT = 200;

        Logger::LogLevel logLevel = Logger::WARN;
        lv_obj_t* toastContainer;

        std::vector<ActiveToast> activeToasts = {};
    };
};
