#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * 
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 * TODO: Automatically detect which robot is plugged in and set the robot variable to the correct robot class.
 */
void initialize()
{
    Logger::info("==== Initialize ===="); // <-- Change this to the target robot
    robot = std::make_shared<BlazeRobot>(); // <-- Change this to the target robot

    // Reads the ROBOT_NAME constant from generated_robot.hpp and creates the correct robot 
    // class based on that name. This automatically detects which robot is plugged in and sets 
    // the robot variable to the correct robot class.
    // This probably doesn't work and the files were confusing me

    // if (ROBOT_NAME == "Blaze")
    // {
    //     robot = std::make_shared<BlazeRobot>();
    // }
    // else if (ROBOT_NAME == "PJ")
    // {
    //     robot = std::make_shared<PJRobot>();
    // }
    // else if (ROBOT_NAME == "Robin")
    // {
    //     robot = std::make_shared<RobinRobot>();
    // }
    // else if (ROBOT_NAME == "SwerveTest")
    // {
    //     robot = std::make_shared<SwerveTestRobot>();
    // }
    // else
    // {
    //     Logger::error("Unknown robot name: " + std::string(ROBOT_NAME));
    //     Logger::error("Please check the ROBOT_NAME constant in generated_robot.hpp");
    //     Logger::error("Defaulting to BlazeRobot");
    //     robot = std::make_shared<BlazeRobot>(); 
    // }

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
