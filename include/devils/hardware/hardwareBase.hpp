#pragma once
#include <cstdint>
#include <stdexcept>
#include "../utils/logger.hpp"
#include "pros/apix.h"

namespace devils
{
    /**
     * The status of a hardware operation.
     */
    enum HWStatus
    {
        // success
        SUCCESS,

        // internal
        ERROR_NO_SUCCESSFUL_RESULTS,

        // errno
        ERROR_INVALID_PORT,
        ERROR_INCORRECT_TYPE,
        ERROR_CALIBRATING,
        ERROR_PORT_IN_USE,
        ERROR_ACCESS_DENIED,
        ERROR_TOO_MANY_OPEN_FILES,
        ERROR_NOT_IMPLEMENTED,
        ERROR_OUT_OF_DOMAIN,
        ERROR_OUT_OF_BOUNDS,
        ERROR_IO,
        ERROR_INVALID_ARGUMENT,
        ERROR_OUT_OF_MEMORY,
        ERROR_RESOURCE_BUSY,
        ERROR_UNKNOWN
    };

    /**
     * Represents an error related to hardware components.
     */
    template <typename T = void>
    struct HWResult
    {
        T value;
        HWStatus status;

        // Constructor for implicit conversion from T to HWResult<T>
        HWResult(const T val) :
            value(val),
            status(SUCCESS)
        {
        }

        // Constructor for explicit creation of HWResult with status
        HWResult(const HWStatus status) :
            value(T{}),
            status(status)
        {
        }

        // Conversion operator for implicit conversion from HWResult<T> to T
        operator T() const
        {
            return value;
        }

        /**
         * Checks if the result is successful.
         * @return True if the result is successful, false otherwise.
         */
        bool isSuccess() const
        {
            return status == SUCCESS;
        }
    };

    /**
     * Base class for all hardware devices.
     */
    class HardwareBase
    {
    public:
        /**
         * Base class for all hardware devices.
         * @param name The name of the hardware (e.g. "Left Drive Motor")
         * @param type The type of the hardware (e.g. SmartMotor)
         * @param portName The name of the port the hardware is connected to (e.g. "A")
         */
        HardwareBase(
            std::string name,
            std::string type,
            std::string portName)
            : name(std::move(name)),
              type(std::move(type)),
              portName(std::move(portName))
        {
        }

    protected:

        /**
         * Reports an error based on a hardware status code.
         * @param status The hardware status code to report an error for.
         */
        void reportErrorFromStatus(const HWStatus status) const
        {
            switch (status)
            {
            case ERROR_INCORRECT_TYPE:
                Logger::error(toString() + " isn't connected");
                break;
            case ERROR_INVALID_PORT:
                Logger::error(toString() + " isn't connected to a valid port.");
                break;
            case ERROR_CALIBRATING:
                Logger::error(toString() + " is busy calibrating.");
                break;
            case ERROR_RESOURCE_BUSY:
            case ERROR_PORT_IN_USE:
                Logger::error(toString() + " is ` use by another process.");
                break;
            default:
                Logger::error(toString() + " encountered an unknown error (code " + std::to_string(status) + ").");
                break;
            }
        }

        /**
         * Executes a function and checks for errors based on errno.
         * @tparam ReturnType Return type of the function
         * @tparam Func The function type
         * @tparam Args The argument types
         * @param func The function to execute
         * @param args The arguments to pass to the function
         * @return The result of the function or an error code if an error occurred.
         */
        template<typename ReturnType, typename Func, typename... Args>
        HWResult<ReturnType> executeWithErrorCheck(Func func, Args... args) {
            // Always resets errno before calling the function
            errno = 0;

            // Call the function with its arguments
            ReturnType result = func(args...);
            auto statusCode = getStatusCode();

            // Check errno for errors
            if (statusCode != SUCCESS)
            {
                reportErrorFromStatus(statusCode);
                return statusCode;
            }
            return result; // or handle someValue as needed
        }

        /**
         * Gets the hardware status code based on the current errno value.
         * @return The hardware status code based on the current errno value.
         */
        static HWStatus getStatusCode()
        {
            switch (errno)
            {
            case 0:
                return SUCCESS;
            case ENXIO:
            case ENOENT:
                return ERROR_INVALID_PORT;
            case ENODEV:
                return ERROR_INCORRECT_TYPE;
            case EAGAIN:
                return ERROR_CALIBRATING;
            case EADDRINUSE:
                return ERROR_PORT_IN_USE;
            case EACCES:
                return ERROR_ACCESS_DENIED;
            case ENFILE:
                return ERROR_TOO_MANY_OPEN_FILES;
            case ENOSYS:
                return ERROR_NOT_IMPLEMENTED;
            case EDOM:
                return ERROR_OUT_OF_DOMAIN;
            case EOVERFLOW:
                return ERROR_OUT_OF_BOUNDS;
            case EIO:
                return ERROR_IO;
            case EINVAL:
                return ERROR_INVALID_ARGUMENT;
            case ENOMEM:
                return ERROR_OUT_OF_MEMORY;
            case EBUSY:
                return ERROR_RESOURCE_BUSY;
            default:
                return ERROR_UNKNOWN;
            }
        }

        /**
         * Returns a string representation of the hardware, including its name and port.
         * @return A string representation of the hardware, including its name and port (e.g. "Left Drive Motor (port A)").
         */
        std::string toString() const
        {
            return name + " (port " + portName + ")";
        }

        std::string name;
        std::string type;
        std::string portName;
    };

    /**
     * Represents a `HardwareBase` that is connected to a specific ADI port on the VEX V5 brain.
     */
    class ADIHardwareBase : public HardwareBase
    {
    public:
        ADIHardwareBase(std::string name, std::string type, const char port)
            : HardwareBase(std::move(name), std::move(type), adiPortToString(port)),
              port(port)
        {
        }

    protected:
        char port;

    private:
        /**
         * Converts an ADI port id (e.g. 'A', 'B', etc.) to its corresponding string representation (e.g. "A", "B", etc.).
         * @param port - The ID of the port (e.g. 'A', 'B', etc.)
         * @return A string representation of the ADI port (e.g. "A", "B", etc.). If the port name is invalid, returns "?".
         */
        static std::string adiPortToString(const char port)
        {
            if (port >= 'A' && port <= 'H')
                return std::string(1, port); // Convert 'A'-'H' to "A"-"H"
            if (port >= 'a' && port <= 'h')
                return std::string(1, port - ('a' - 'A')); // Convert 'a'-'h' to "A"-"H"

            // Invalid port name
            return "?";
        }
    };

    /**
     * Represents a `HardwareBase` that is connected to a specific V5 port on the VEX V5 brain.
     */
    class V5HardwareBase : public HardwareBase
    {
    public:
        V5HardwareBase(
            std::string name,
            std::string type,
            const uint8_t port)
            : HardwareBase(std::move(name), std::move(type), std::to_string(port)),
              port(port)
        {
        }
    protected:
        uint8_t port;
    };
}
