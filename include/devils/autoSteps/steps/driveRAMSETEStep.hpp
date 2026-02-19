#pragma once

#include <utility>
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../geometry/units.hpp"
#include "../../geometry/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../trajectory/trajectory.hpp"

namespace devils
{
    /**
     * Drives the robot along a trajectory using RAMSETE control.
     * See: https://file.tavsys.net/control/controls-engineering-in-frc.pdf
     */
    class DriveRAMSETEStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief A proportional constant. Must be greater than 0. Larger values will result in more aggressive control.
            float proportionGain = 0.005;

            /// @brief A damping coefficient. Must be between 0 and 1. Larger values will result in increased damping.
            float dampingCoefficient = 0.75;

            /// @brief Proportion (P in PID) between translational velocity and motor voltage
            float translationP = 0.015;

            /// @brief Proportion (P in PID) between rotational velocity and motor voltage
            float rotationP = 0.1;

            /// @brief Time in seconds of odometry latency. This is the time it takes for the odometry to update after the robot moves.
            float sensorLatency = 0.1;

            /// @brief The default options for the drive step.
            static Options defaultOptions;
        };

        /**
         * Drives the robot along a trajectory using ramsete control.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param trajectory The trajectory to follow.
         * @param options The options for the drive step.
         */
        DriveRAMSETEStep(
            ChassisBase& chassis,
            OdomSource& odomSource,
            const std::shared_ptr<Trajectory>& trajectory,
            const Options& options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              trajectory(trajectory),
              internalTimer(trajectory->duration()),
              options(options)
        {
        }

    protected:
        void onStart() override
        {
            internalTimer.start();
        }

        void onUpdate() override
        {
            // Get the current time
            const auto t = internalTimer.getElapsedTime();

            // Get current setpoint
            const auto setpoint = trajectory->getStateAt(t);
            const auto feedbackSetpoint = trajectory->getStateAt(t - options.sensorLatency);

            // Get current position
            const auto currentPosition = odomSource.getPose();

            // Calculate error to setpoint
            const auto error = feedbackSetpoint.pose - currentPosition;
            
            Logger::debug(error.toString());

            // Calculate local error
            // This is the error relative to the robot's current rotation
            // such that localError.x is aligned with the front of the robot and
            // localError.y is aligned with the side of the robot.
            const auto localError = Pose(
                error.x * cosf(currentPosition.rotation) + error.y * sinf(currentPosition.rotation),
                -error.x * sinf(currentPosition.rotation) + error.y * cosf(currentPosition.rotation),
                Units::diffRad(feedbackSetpoint.pose.rotation, currentPosition.rotation));

            // Calculate controller gain
            // k = 2 * zeta * sqrt(w^2 + b * v^2)
            float controllerGain = setpoint.angularVelocity * setpoint.angularVelocity +
                options.proportionGain * setpoint.velocity * setpoint.velocity;
            controllerGain = 2 * options.dampingCoefficient * std::sqrt(controllerGain);

            // Calculate sinc (sin(x) / x)
            float sinc = sinf(localError.rotation) / localError.rotation;
            if (std::isnan(sinc))
                sinc = 1.0; // Handle the case where localError.rotation is 0

            // Calculate translation output
            // v = v_d * cos(e_r) + k * e_x
            float translationOutput = setpoint.velocity * cosf(localError.rotation) +
                controllerGain * localError.x;

            // Calculate rotation output
            // w = w_d + k * e_r + b * v * sinc(e_r) * e_y
            float rotationOutput = setpoint.angularVelocity +
                controllerGain * localError.rotation +
                options.proportionGain * setpoint.velocity * sinc * localError.y;
            
            // Multiply by translation and rotation P values
            translationOutput *= options.translationP;
            rotationOutput *= options.rotationP;

            // Set the chassis output
            chassis.move(translationOutput, rotationOutput, 0.0f);
        }

        void onStop() override
        {
            chassis.stop();
        }

        bool checkFinished() override
        {
            return internalTimer.getIsFinished();
        }

        ChassisBase& chassis;
        OdomSource& odomSource;
        std::shared_ptr<Trajectory> trajectory;
        Timer internalTimer;
        Options options;
    };
}

// Define the default options
devils::DriveRAMSETEStep::Options devils::DriveRAMSETEStep::Options::defaultOptions = Options();
