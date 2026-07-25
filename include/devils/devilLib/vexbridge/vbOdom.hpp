#pragma once
#include "../odom/odomSource.hpp"
#include "../geometry/units.hpp"
#include "./vexbridge.h"

namespace devils
{
    /**
     * Syncs odometry data to VEXBridge.
     */
     // TODO: This used to use "Runnable" instead of "AsyncTask". It would seem that AsyncTask is the newer version of Runnable,
     // So I swapped it in but I think this is what crashes the brain.
    class VBOdom : private AsyncTask
    {
    public:
        /**
         * Creates a new odometry syncer.
         * @param name The name of the odometry data.
         * @param odometry The odometry source to sync.
         */
        VBOdom(std::string name, OdomSource &odometry)
            : odometry(odometry),
              group("_poses/" + name),
              xValue(group.addValue("x", 0.0f)),
              yValue(group.addValue("y", 0.0f)),
              rotation(group.addValue("rotation", 0.0f)),
              width(group.addValue("width", 15.0f)),
              length(group.addValue("length", 15.0f)),
              speed(group.addValue("speed", 0.0f))
        {
            this->start();
        }

        /**
         * Sets the dimensions of the robot.
         * @param width The width of the robot in inches.
         * @param length The length of the robot in inches.
         */
        void setDimensions(float width, float length)
        {
            this->width.set(width);
            this->length.set(length);
        }

    protected:
        void onUpdate() override
        {
            Pose pose = odometry.getPose();
            xValue.set(pose.x);
            yValue.set(pose.y);
            rotation.set(Units::radToDeg(pose.rotation));
            speed.set(odometry.getVelocity().magnitude());
        }

    private:
        VBGroup group;
        VBValue<float> xValue;
        VBValue<float> yValue;
        VBValue<float> rotation;
        VBValue<float> width;
        VBValue<float> length;
        VBValue<float> speed;
        OdomSource &odometry;
    };
}