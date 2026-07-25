#pragma once
#include "../odom/odomSource.hpp"
#include "../geometry/units.hpp"
#include "./vexbridge.h"

namespace devils
{
    /**
     * Syncs swerve data to VEXBridge.
     */
     // TODO: This used to use "Runnable" instead of "AsyncTask". It would seem that AsyncTask is the newer version of Runnable,
     // So I swapped it in but I think this is what crashes the brain.
    class VBSwerve : private AsyncTask
    {
    public:
        /**
         * Creates a new swerve syncer.
         * @param name The name of the swerve data object to display in VEXBridge.
         * @param chassis The swerve chassis to sync.
         */
        VBSwerve(std::string name, SwerveChassis &chassis)
            : odometry(odometry),
              rootGroup(name),
              forward(rootGroup.addValue("Forward Input", 0.0f)),
              strafe(rootGroup.addValue("Strafe Input", 0.0f)),
              turn(rootGroup.addValue("Turn Input", 0.0f)),

              fl("Front Left Module"),
              flTargetAngle(fl.addValue("FL Target Angle", 0.0f)),
              flCurrentAngle(fl.addValue("FL Current Angle", 0.0f)),
              flError(fl.addValue("FL Error", 0.0f)),
              flSpeed(fl.addValue("FL Speed", 0.0f)),
              fr("Front Right Module"),
              frTargetAngle(fr.addValue("FR Target Angle", 0.0f)),
              frCurrentAngle(fr.addValue("FR Current Angle", 0.0f)),
              frError(fr.addValue("FR Error", 0.0f)),
              frSpeed(fr.addValue("FR Speed", 0.0f)),
              bl("Back Left Module"),
              blTargetAngle(bl.addValue("BL Target Angle", 0.0f)),
              blCurrentAngle(bl.addValue("BL Current Angle", 0.0f)),
              blError(bl.addValue("BL Error", 0.0f)),
              blSpeed(bl.addValue("BL Speed", 0.0f)),
              br("Back Right Module"),
              brTargetAngle(br.addValue("BR Target Angle", 0.0f)),
              brCurrentAngle(br.addValue("BR Current Angle", 0.0f)),
              brError(br.addValue("BR Error", 0.0f)),
              brSpeed(br.addValue("BR Speed", 0.0f))

        {
            this->start();
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
        SwerveChassis &chassis;

        VBGroup rootGroup;
        VBValue<float> forward;
        VBValue<float> strafe;
        VBValue<float> turn;

        VBGroup fl;
        VBValue<float> flTargetAngle;
        VBValue<float> flCurrentAngle;
        VBValue<float> flError;
        VBValue<float> flSpeed;
        VBGroup fr;
        VBValue<float> frTargetAngle;
        VBValue<float> frCurrentAngle;
        VBValue<float> frError;
        VBValue<float> frSpeed;
        VBGroup bl;
        VBValue<float> blTargetAngle;
        VBValue<float> blCurrentAngle;
        VBValue<float> blError;
        VBValue<float> blSpeed;
        VBGroup br;
        VBValue<float> brTargetAngle;
        VBValue<float> brCurrentAngle;
        VBValue<float> brError;
        VBValue<float> brSpeed;
    };
}