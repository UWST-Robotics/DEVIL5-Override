#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * 
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    Logger::info("==== Initialize ====");
    //robot = std::make_shared<BlazeRobot>(); // <-- Change this to the target robot

    // Makes a new card reader and reads the robot name from the card.
    // This is used to set the robot name in the generated header file.
    // Then uses the robot name to upload the correct robot data.
    devils::SDCardHandler cardHandler;
    std::string robotName = cardHandler.readFile("robotName.txt"); // IMPORTANT: This file must be on the SD card for the robot to work correctly. It should contain the name of the robot (BLAZE, PJ, ROBIN, etc.)
    if (robotName == "BLAZE")
        robot = std::make_shared<BlazeRobot>();
    else if (robotName == "PJ")
        robot = std::make_shared<PJRobot>();
    else if (robotName == "ROBIN")
        robot = std::make_shared<RobinRobot>();
    else
    {
        Logger::warn("Robot name not found on SD card. Using default name.");
        robotName = "ROBOT NAME NOT FOUND";
        robot = std::make_shared<BlazeRobot>(); // Change this to name if robot does not have an SD card. This is a fallback to prevent the robot from crashing.
    }

    Logger::info("Robot created");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */ 
void disabled()
{
    Logger::info("==== Disabled ====");
    robot->disabled();
    Logger::info("Robot disabled");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
    Logger::info("==== Competition Initialize ====");
    robot->competition();
    Logger::info("Competition initialized");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    Logger::info("==== Autonomous ====");
    robot->autonomous();
    Logger::info("Autonomous complete");
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
    Logger::info("==== Teleoperated ====");
    robot->opcontrol();
    Logger::info("Teleoperated complete");
}
