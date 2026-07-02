#pragma once

#include "chassisBase.hpp"
#include "../utils/backgroundService.hpp"
#include "../odom/odomSource.hpp"
#include "../odom/poseVelocityCalculator.hpp"
#include "../geometry/math.hpp"

namespace devils
{
    /**
     * Represents a chassis that maintains a virtual position and orientation.
     * This is useful for testing autonomous routines without a physical robot.
     * Can be used as an OdomSource.
     */
    class DummyChassis :
        public BackgroundService,
        public ChassisBase,
        public OdomSource,
        public PoseVelocityCalculator
    {
    public:
        /**
         * Moves the robot in a direction using voltage.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         * @param strafe The strafe speed of the robot from -1 to 1.
         */
        void move(float forward, float turn, float strafe) override
        {
            forward = std::clamp(forward, -1.0f, 1.0f);
            turn = std::clamp(turn, -1.0f, 1.0f);
            strafe = std::clamp(strafe, -1.0f, 1.0f);

            lastForward = forward;
            lastTurn = turn;
            lastStrafe = strafe;
        }

        /**
         * Jumps the robot to a specific pose.
         * @param pose The pose to set the robot to.
         */
        void setPose(const Pose pose) override
        {
            currentPose = pose;
        }

        /**
         * Gets the current pose of the robot.
         * @return The current pose of the robot.
         */
        Pose getPose() override
        {
            return currentPose;
        }

        /**
         * Gets the current velocity of the robot.
         * @return Current velocity of the robot.
         */
        PoseVelocity getVelocity() override
        {
            return PoseVelocityCalculator::getVelocity();
        }

        /**
         * Checks if the chassis is holonomic, meaning it can strafe in any direction.
         * @return True if the chassis is holonomic, false otherwise.
         */
        bool isHolonomic() const override
        {
            return holonomicEnabled;
        }

        /**
         * Sets whether the chassis is holonomic, meaning it can strafe in any direction.
         * @param holonomic True if the chassis should be holonomic, false otherwise.
         */
        void setHolonomic(const bool holonomic)
        {
            holonomicEnabled = holonomic;
        }

    protected:
        void onUpdate() override
        {
            // TODO: Multiply acceleration by delta time

            // Get Strafe Input
            const float strafeInput = holonomicEnabled ? lastStrafe : 0;

            // Calculate Acceleration
            currentAcceleration.x += TRANSLATION_ACCEL * (
                cosf(currentPose.rotation) * lastForward +
                sinf(currentPose.rotation) * strafeInput);

            currentAcceleration.y += TRANSLATION_ACCEL * (
                sinf(currentPose.rotation) * lastForward +
                cosf(currentPose.rotation) * strafeInput);

            currentAcceleration.rotation += lastTurn * ROTATION_ACCEL;
            currentAcceleration = currentAcceleration * (1 - DRAG);

            // Update Pose
            currentPose = currentPose + currentAcceleration;
            currentPose.rotation = fmodf(currentPose.rotation, 2 * M_PIF);

            // Update Velocity
            updateVelocity(currentPose);
        }

    private:
        static constexpr float TRANSLATION_ACCEL = 0.8; // in/s^2
        static constexpr float ROTATION_ACCEL = 0.1; // rad/s^2
        static constexpr float DRAG = 0.35; // %

        float lastForward = 0;
        float lastTurn = 0;
        float lastStrafe = 0;
        Pose currentAcceleration = Pose();
        Pose currentPose = Pose();
        bool holonomicEnabled = true;
    };
}
