#pragma once

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

/*
 *      Headers defined in this file are accessible from anywhere in the project.
 */

// Chassis
#include "chassis/chassisBase.hpp"
#include "chassis/tankChassis.hpp"
#include "chassis/mecanumChassis.hpp"
#include "chassis/hDriveChassis.hpp"
#include "chassis/xDriveChassis.hpp"
#include "chassis/dummyChassis.hpp"
#include "chassis/swerveChassis.hpp"

// Hardware
#include "hardware/inertialSensor.hpp"
#include "hardware/opticalSensor.hpp"
#include "hardware/adiPneumatic.hpp"
#include "hardware/adiDigitalInput.hpp"
#include "hardware/adiAnalogInput.hpp"
#include "hardware/adiPneumaticGroup.hpp"
#include "controller/chainLoop.hpp"

// Odom
#include "odom/odomSource.hpp"
#include "odom/differentialWheelOdom.hpp"
#include "odom/tankChassisOdom.hpp"
#include "odom/perpendicularSensorOdom.hpp"
#include "odom/parallelSensorOdom.hpp"
#include "odom/delayedOdom.hpp"

// Utils
#include "utils/robot.hpp"
#include "utils/timer.hpp"
#include "utils/stopwatch.hpp"

// Geometry
#include "geometry/lerp.hpp"
#include "geometry/math.hpp"
#include "geometry/pose.hpp"
#include "geometry/poseVelocity.hpp"
#include "geometry/units.hpp"
#include "geometry/vector2.hpp"
#include "geometry/vector3.hpp"

// Path
#include "path/path.hpp"
#include "path/linearPath.hpp"
#include "path/splinePath.hpp"

// Trajectory
#include "trajectory/trajectory.hpp"
#include "trajectory/trajectoryGenerator.hpp"
#include "trajectory/trajectoryConstraints.hpp"

// Controller
#include "controller/pidController.hpp"
#include "controller/armFeedforward.hpp"
#include "controller/elevatorFeedforward.hpp"
#include "controller/motorFeedforward.hpp"
#include "controller/trapezoidMotionProfile.hpp"

// AutoSteps
#include "autoSteps/autoBuilder.hpp"
#include "autoSteps/steps/drivePIDStep.hpp"
#include "autoSteps/steps/driveRAMSETEStep.hpp"
#include "autoSteps/steps/rotateMotionProfileStep.hpp"

// Display
#include "display/toastDisplay.hpp"
#include "display/devilbotsDisplay.hpp"
#include "display/autoPickerDisplay.hpp"
// #include "display/eyesRenderer.hpp"

// VEXBridge
// TODO: VEXBridge is currently broken
#include "vexbridge/vexbridge.h"
#include "vexbridge/VBOdom.hpp"
