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
    class InertialSensor : HardwareBase, public IGyro
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
            const int8_t port)
            : HardwareBase(name, "IMU", port),
              imu(port)
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
        void calibrate() const
        {
            const auto status = imu.reset(false);
            if (status == PROS_ERR)
                reportError();
        }

        /**
         * Waits until the IMU is finished calibrating.
         * Should be run to avoid movement during calibration.
         */
        void waitUntilCalibrated() const
        {
            while (imu.is_calibrating())
                pros::delay(20);
        }

        /**
         * Gets the current acceleration of the IMU in inches per second squared.
         * @return The current acceleration of the IMU in inches per second squared.
         */
        HWResult<Vector3> getAccel() const
        {
            const auto accel = imu.get_accel();
            const auto x = static_cast<float>(accel.x);
            const auto y = static_cast<float>(accel.y);
            const auto z = static_cast<float>(accel.z);

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
        HWResult<float> getRawHeading() const
        {
            const auto heading = static_cast<float>(imu.get_rotation());
            if (heading == PROS_ERR_F)
                return getStatusCode();

            // Apply scale/offset
            return Units::degToRad(heading);
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
        HWResult<float> getPitch() const
        {
            const auto pitch = static_cast<float>(imu.get_pitch());
            if (pitch == PROS_ERR_F)
                return getStatusCode();
            return Units::degToRad(pitch);
        }

        /**
         * Gets the current roll of the IMU in radians.
         * @return The current roll of the IMU in radians or 0 if the operation failed.
         */
        HWResult<float> getRoll() const
        {
            const auto roll = static_cast<float>(imu.get_roll());
            if (roll == PROS_ERR_F)
                return getStatusCode();
            return Units::degToRad(roll);
        }

        /**
         * Gets the current yaw of the IMU in radians.
         * @return The current yaw of the IMU in radians or 0 if the operation failed.
         */
        HWResult<float> getYaw() const
        {
            const auto yaw = static_cast<float>(imu.get_yaw());
            if (yaw == PROS_ERR_F)
                return getStatusCode();
            return Units::degToRad(yaw);
        }

    private:
        float headingScale = 1;
        float headingOffset = 0;
        bool isCalibrating = false;
        bool isErrored = false;
        bool isConnected = false;

        pros::IMU imu;
        Pose odomPose;
    };
}
