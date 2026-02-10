#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <format>
#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents a global logging utility.
     */
    class Logger
    {
    public:
        /// @brief The log level (INFO, WARN, ERROR, DEBUG).
        enum LogLevel
        {
            INFO,
            WARN,
            ERROR,
            DEBUG
        };

        /**
         * Logs a message to the terminal, display, SD card, and network.
         * @param message The message to log.
         * @param level The log level.
         */
        static void log(
            const std::string& message,
            const LogLevel level)
        {
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
            const auto time = pros::millis();
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

        /**
         * Logs an info message.
         * @param message The message to log.
         */
        static void info(std::string message)
        {
            log(message, INFO);
        }

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        static void warn(std::string message)
        {
            log(message, WARN);
        }

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        static void error(std::string message)
        {
            log(message, ERROR);
        }

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        static void debug(std::string message)
        {
            log(message, DEBUG);
        }

    private:
        Logger() = delete;
    };
}
