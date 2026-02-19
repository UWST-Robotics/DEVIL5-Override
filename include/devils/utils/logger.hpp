#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <format>
#include <stack>

#include "pros/rtos.hpp"

namespace devils
{
    // Forward declare `ToastDisplay` to prevent circular dependency
    class ToastDisplay;
    
    /**
     * Represents a global logging utility.
     */
    class Logger
    {
    public:
        // Delete constructor to prevent instantiation
        Logger() = delete;
        
        /// @brief The log level (INFO, WARN, ERROR, DEBUG).
        enum LogLevel
        {
            // Least severe
            DEBUG = 0,
            INFO,
            WARN,
            ERROR,
            // Most severe
        };
        
        /// @brief Represents a single log message.
        struct LogMessage
        {
            uint32_t timestamp;
            LogLevel level;
            std::string text;
        };

        /**
         * Logs an info message.
         * @param message The message to log.
         */
        static void info(const std::string& message)
        {
            log({pros::millis(), INFO, message});
        }

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        static void warn(const std::string& message)
        {
            log({pros::millis(), WARN, message});
        }

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        static void error(const std::string& message)
        {
            log({pros::millis(), ERROR, message});
        }

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        static void debug(const std::string& message)
        {
            log({pros::millis(), DEBUG, message});
        }
        
        /**
         * Logs a message to the terminal and the `ToastDisplay` (if it exists).
         * @param logMessage The log message to log.
         */
        static void log(const LogMessage& logMessage)
        {
            printToConsole(logMessage);
            displayToast(logMessage);
            // TODO: Implement SD Card Logging
        }
        
    protected:
        /**
         * Adds a log message to the toast buffer to be displayed as a toast notification on the `ToastDisplay`.
         * If the buffer exceeds `MAX_BUFFER_SIZE`, the message is discarded to prevent memory issues.
         * @param logMessage - The log message to add to the toast buffer.
         */
        static void displayToast(const LogMessage& logMessage)
        {
            if (toastBuffer.size() < MAX_BUFFER_SIZE)
                toastBuffer.push(logMessage);
        }
        
        /**
         * Logs a message to the serial console.
         * Can be accessed by used `pros terminal` on a connected device
         * while connected to the VEX V5 brain (or controller) over USB.
         * @param logMessage - The log message to print to the console.
         */
        static void printToConsole(const LogMessage& logMessage)
        {
            // Get the message and level from the log message struct
            const auto time = logMessage.timestamp;
            const auto message = logMessage.text;
            const auto level = logMessage.level;
            
            // ANSI escape codes (prefix colors)
            if (level == INFO)
                std::cout << "\033[1;94m";
            else if (level == WARN)
                std::cout << "\033[38;5;11m";
            else if (level == ERROR)
                std::cout << "\033[38;5;160m";
            else if (level == DEBUG)
                std::cout << "\033[1;90m";

            // Timestamp
            const auto milliseconds = time % 1000;
            const auto seconds = time / 1000;
            const auto minutes = seconds / 60;
            std::cout << std::format("{:02}:{:02}.{:03}", minutes, seconds % 60, milliseconds) << " |";

            // Log level
            if (level == INFO)
                std::cout << " INFO : ";
            else if (level == WARN)
                std::cout << " WARN : ";
            else if (level == ERROR)
                std::cout << " ERROR : ";
            else if (level == DEBUG)
                std::cout << " DEBUG : ";
            
            // ANSI escape codes (message colors)
            if (level == WARN)
                std::cout << "\033[38;5;11m";
            else if (level == ERROR)
                std::cout << "\033[38;5;160m";

            // Message
            std::cout << message << "\033[0m" << std::endl;
        }
    private:
        /// @brief Represents a buffer of log messages that have been logged but not yet displayed as toasts.
        static inline std::stack<LogMessage> toastBuffer = {};
        static constexpr int MAX_BUFFER_SIZE = 1024;
        friend class ToastDisplay;
    };
}
