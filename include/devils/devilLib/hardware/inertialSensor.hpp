#pragma once

#include "pros/imu.hpp"
#include "pros/error.h"
#include "gyroBase.h"
#include "hardwareBase.hpp"
#include "../geometry/units.hpp"
#include "../geometry/vector3.hpp"
#include "../odom/odomSource.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 inertial measurement unit.
     */
    class InertialSensor : V5HardwareBase, public IGyro
    {
    public:
        /**
         * Creates a new IMU.
         * Remember to calibrate the IMU before use.
         * @param name The name of the IMU (for logging purposes)
         * @param port The port of the IMU (from 1 to 21)
         */
        InertialSensor(
            const std::string& name,
            const uint8_t port)
            : V5HardwareBase(name, "IMU", port)
        {
        }

        /**
         * Sets the current heading of the IMU in radians.
         * @param heading The heading to set the IMU to in radians.
         */
        void setHeading(const float heading) override
        {
            const auto rawHeading = getRawHeading();
            if (!rawHeading.isSuccess())
                return;

            headingOffset = heading - rawHeading;
        }

        /**
         * Scales the heading by a given factor.
         * Used to fix consistent heading drift after a set rotation.
         * Can be calculated by rotating the robot exactly 360 degrees and doing `2 * PI / getHeading()`.
         * @param scale The scale to multiply the heading by.
         */
        void setHeadingScale(const float scale)
        {
            headingScale = scale;
        }

        /**
         * Calibrates the IMU. Robot should be still during calibration.
         * Run `waitUntilCalibrated` to wait until calibration is finished.
         */
        void calibrate()
        {
            executeWithErrorCheck<int32_t>(pros::c::imu_reset, port);
        }

        /**
         * Waits until the IMU is finished calibrating.
         * Should be run to avoid movement during calibration.
         */
        void waitUntilDoneCalibrated()
        {
            while (getIsCalibrating())
                pros::delay(20);
        }

        /**
         * Checks if the IMU is currently calibrating.
         * @return True if the IMU is currently calibrating, false otherwise. If the operation failed, an error code is returned.
         */
        HWResult<bool> getIsCalibrating()
        {
            const auto result = executeWithErrorCheck<pros::imu_status_e_t>(pros::c::imu_get_status, port);
            if (!result.isSuccess())
                return result.status;

            return result.value & IMU_STATUS_CALIBRATING;
        }
        
        /**
         * Checks if the IMU is ready to be used (e.g. finished calibrating).
         * @return True if the IMU is ready to be used, false otherwise. If the operation failed, false is returned.
         */
        bool getIsReady() override
        {
            const auto isCalibrating = getIsCalibrating();
            if (!isCalibrating.isSuccess())
                return false;
            return !isCalibrating.value;
        }

        /**
         * Gets the current acceleration of the IMU in inches per second squared.
         * @return The current acceleration of the IMU in inches per second squared.
         */
        HWResult<Vector3> getAccel()
        {
            const auto result = executeWithErrorCheck<pros::imu_accel_s_t>(pros::c::imu_get_accel, port);
            if (!result.isSuccess())
                return result.status;

            const auto x = static_cast<float>(result.value.x);
            const auto y = static_cast<float>(result.value.y);
            const auto z = static_cast<float>(result.value.z);

            if (x == PROS_ERR_F)
                return getStatusCode();

            return Vector3(
                Units::metersToIn(x),
                Units::metersToIn(y),
                Units::metersToIn(z));
        }

        /**
         * Gets the current heading of the IMU in radians, unscaled and without offset.
         * @return The current heading of the IMU in radians or 0 if the operation failed.
         */
        HWResult<float> getRawHeading()
        {
            const auto result = executeWithErrorCheck<double>(pros::c::imu_get_rotation, port);
            if (!result.isSuccess())
                return result.status;

            // Apply scale/offset
            return Units::degToRad(static_cast<float>(result.value));
        }

        /**
         * Gets the current heading of the IMU in radians, unbounded.
         * @return The current heading of the IMU in radians or 0 if the operation failed.
         */
        HWResult<float> getHeading() override
        {
            const auto rawHeading = getRawHeading();
            if (!rawHeading.isSuccess())
                return rawHeading;

            return rawHeading * headingScale + headingOffset;
        }

        /**
         * Gets the current pitch of the IMU in radians.
         * @return The current pitch of the IMU in radians or 0 if the operation failed.
         */
        HWResult<float> getPitch()
        {
            const auto result = executeWithErrorCheck<double>(pros::c::imu_get_pitch, port);
            if (!result.isSuccess())
                return result.status;

            return Units::degToRad(static_cast<float>(result.value));
        }

        /**
         * Gets the current roll of the IMU in radians.
         * @return The current roll of the IMU in radians or 0 if the operation failed.
         */
        HWResult<float> getRoll()
        {
            const auto result = executeWithErrorCheck<double>(pros::c::imu_get_roll, port);
            if (!result.isSuccess())
                return result.status;

            return Units::degToRad(static_cast<float>(result.value));
        }

        /**
         * Gets the current yaw of the IMU in radians.
         * @return The current yaw of the IMU in radians or 0 if the operation failed.
         */
        HWResult<float> getYaw()
        {
            const auto result = executeWithErrorCheck<double>(pros::c::imu_get_yaw, port);
            if (!result.isSuccess())
                return result.status;

            return Units::degToRad(static_cast<float>(result.value));
        }

    private:
        float headingScale = 1;
        float headingOffset = 0;
    };
}
