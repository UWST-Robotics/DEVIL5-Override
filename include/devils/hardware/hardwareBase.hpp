#pragma once
#include <cstdint>
#include <stdexcept>

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
        HWResult(const T& val) : value(val), status(SUCCESS)
        {
        }

        // Constructor for explicit creation of HWResult with status
        HWResult(const HWStatus status) : status(status)
        {
        }

        // Conversion operator for implicit conversion from HWResult<T> to T
        operator T() const
        {
            if (status != SUCCESS)
                throw std::runtime_error("Cannot convert HWResult to T: operation was not successful.");
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
         * @param name The name of the hardware
         * @param type The type of the hardware (e.g. SmartMotor)
         * @param port The port of the hardware
         */
        HardwareBase(
            std::string name,
            std::string type,
            const int8_t port)
            : name(std::move(name)),
              type(std::move(type)),
              port(port)
        {
        }

    protected:
        /**
         * Reports an error based on the current errno value.
         */
        void reportError() const
        {
            reportError(getStatusCode());
        }

        /**
         * Reports an error with a specific status code.
         * @param status The hardware status code to report.
         */
        void reportError(const HWStatus status) const
        {
            // TODO: Implement logging or error reporting mechanism
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

        std::string name;
        std::string type;
        int8_t port;
    };
}
