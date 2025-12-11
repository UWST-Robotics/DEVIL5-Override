#pragma once

#include "devils/devils.h"
#include <algorithm>

namespace devils
{
    /// @brief Struct to hold motor speeds for the intake system.
    struct IntakeMotorSpeeds
    {
        float topSpeed;
        float middleSpeed;
        float bottomSpeed;
    };

    /// @brief Which mode to color sort
    enum ColorSortingMode
    {
        DontSort,
        ScoreRedOnly,
        ScoreBlueOnly
    };

    /// @brief The current block color
    enum BlockColor
    {
        Red,
        Blue
    };

    /// @brief Where the block should be scored
    enum IntakeMode
    {
        /// @brief Cycles the block throughout storage
        Cycler,

        /// @brief The long goals on the top and bottom of the field
        SideGoal,

        /// @brief The top of the goal in the center of the field
        MidTop,

        /// @brief The bottom of the goal in the center of the field
        MidBottom,
    };

    /**
     * Represents the intake arm and claw system of the robot.
     */
    class IntakeSystem
    {
    public:
        IntakeSystem(SmartMotorGroup &frontBottomIntakeMotors,
                     SmartMotorGroup &frontTopIntakeMotors,
                     SmartMotorGroup &frontIntakeRollers,
                     SmartMotorGroup &backIntakeMotors,
                     SmartMotorGroup &cyclerMotors,
                     OpticalSensor &colorSensor,
                     ADIPneumatic &intakePneumaticsLeft,
                     ADIPneumatic &intakePneumaticsRight,
                     ADIPneumatic &hoodPneumatics)
            : frontBottomIntakeMotors(frontBottomIntakeMotors),
              frontTopIntakeMotors(frontTopIntakeMotors),
              frontIntakeRollers(frontIntakeRollers),
              backIntakeMotors(backIntakeMotors),
              cyclerMotors(cyclerMotors),
              colorSensor(colorSensor),
              intakePneumaticsLeft(intakePneumaticsLeft),
              intakePneumaticsRight(intakePneumaticsRight),
              hoodPneumatics(hoodPneumatics)

        {
            // Reset Position of all motors
            frontBottomIntakeMotors.setPosition(0);
            frontTopIntakeMotors.setPosition(0);
            frontIntakeRollers.setPosition(0);
            backIntakeMotors.setPosition(0);
            cyclerMotors.setPosition(0);

            // Max Brightness
            colorSensor.setLEDBrightness(100);
        }

        /**
         * Moves intake using speed
         * @param speed - Voltage to run intake between -1 and 1.
         */
        void runIntake(float speed)
        {
            speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);

            switch (intakeMode)
            {
            case IntakeMode::Cycler:
                frontBottomIntakeMotors.moveVoltage(speed);
                frontTopIntakeMotors.moveVoltage(speed);
                frontIntakeRollers.moveVoltage(speed);
                // Slower speed is due to the diameter of the back intake being larger than the cycler motors
                // Absolute value is used to ensure the back intake *always* moves in the same direction
                backIntakeMotors.moveVoltage(std::fabs(speed * INTAKE_BACK_SPEED));
                cyclerMotors.moveVoltage(speed);

                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(false);
                break;

            case IntakeMode::SideGoal:
                frontBottomIntakeMotors.moveVoltage(speed);
                frontTopIntakeMotors.moveVoltage(speed);
                frontIntakeRollers.moveVoltage(speed);
                backIntakeMotors.moveVoltage(speed);
                cyclerMotors.moveVoltage(-speed);
                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(true);
                break;

            case IntakeMode::MidTop:
                frontBottomIntakeMotors.moveVoltage(-speed);
                frontTopIntakeMotors.moveVoltage(speed);
                frontIntakeRollers.moveVoltage(speed);
                backIntakeMotors.moveVoltage(speed);
                cyclerMotors.moveVoltage(-speed);

                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(true);
                break;

            case IntakeMode::MidBottom:
                frontBottomIntakeMotors.moveVoltage(speed);
                frontTopIntakeMotors.moveVoltage(-speed);
                frontIntakeRollers.moveVoltage(speed);
                backIntakeMotors.moveVoltage(speed);
                cyclerMotors.moveVoltage(-speed);

                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(true);
                break;
            }
        }

        /**
         * Extends/retracts the intake arms
         * @param extended - Whether the arms should be extended
         */
        void setArmsExtended(bool extended)
        {
            intakePneumaticsLeft.setExtended(extended);
            intakePneumaticsRight.setExtended(extended);
        }

        /**
         * Sets the color sorting mode
         * @param mode - How the intake hood should sort balls
         */
        void setColorSortingMode(ColorSortingMode mode)
        {
            sortingMode = mode;
        }

        /**
         * Sets the intake mode
         * @param mode - How the intake should operate
         */
        void setIntakeMode(IntakeMode mode)
        {
            intakeMode = mode;
        }

        /**
         * Sets the intake hood to be automatic or manual.
         * If automatic, the hood will automatically move to the correct position.
         * If manual, the hood will not move unless explicitly told to do so by `setHoodExtended()`.
         * @param isAutomatic - If true, the hood will automatically move.
         */
        void setHoodAutomatic(bool isAutomatic)
        {
            isHoodAutomatic = isAutomatic;
        }

        /**
         * Sets the intake hood to be extended or retracted.
         * Disables automatic hood control.
         * @param isExtended - If true, goes to score. Otherwise, goes to storage.
         */
        void setHoodExtended(bool isExtended)
        {
            hoodPneumatics.setExtended(isExtended);
            isHoodAutomatic = false;
        }

    protected:
        /**
         * Updates the intake hood to handle color sorting
         */
        void setHoodExtendedBasedOnColorSort()
        {
            if (sortingMode == ColorSortingMode::DontSort)
                return;

            BlockColor currentBlockColor = getCurrentBlockColor();

            // Check if we can score either color
            bool canScoreBlue = currentBlockColor == BlockColor::Blue &&
                                sortingMode == ColorSortingMode::ScoreBlueOnly;

            bool canScoreRed = currentBlockColor == BlockColor::Blue &&
                               sortingMode == ColorSortingMode::ScoreRedOnly;

            // If we can score, retract the hood
            // Otherwise, extend the hood
            bool isCorrectSorting = canScoreBlue || canScoreBlue;
            hoodPneumatics.setExtended(!isCorrectSorting);
        }

        /**
         * Gets the current color of the block in the intake.
         * @return The current color of the block in the intake.
         */
        BlockColor getCurrentBlockColor()
        {
            // Gets the current hue of the color sensor
            double hue = colorSensor.getHue();

            // Calculate distance to the color hue (in degrees)
            double redDist = std::abs(hue - RED_BLOCK_HUE);
            double blueDist = std::abs(hue - BLUE_BLOCK_HUE);

            // Gets the block color of the closest hue
            return redDist > blueDist ? BlockColor::Blue : BlockColor::Red;
        }

    private:
        static constexpr float MAX_SPEED = 1.0;  // %
        static constexpr float MIN_SPEED = -0.6; // %

        static constexpr float INTAKE_BACK_SPEED = 0.6; // %

        // static constexpr float OUTTAKE_TOP_SPEED = 0.8;     // %
        // static constexpr float OUTTAKE_MIDDLE_SPEED = -0.5; // %
        // static constexpr float EXIT_CYCLER_SPEED = 0.5;     // %

        // TODO: Get color hue for block
        static constexpr float RED_BLOCK_HUE = 0.0;
        static constexpr float BLUE_BLOCK_HUE = 0.0;

        // State
        ColorSortingMode sortingMode = ColorSortingMode::DontSort;
        IntakeMode intakeMode = IntakeMode::Cycler;
        bool isHoodAutomatic = true;

        // Hardware
        SmartMotorGroup &frontBottomIntakeMotors;
        SmartMotorGroup &frontTopIntakeMotors;
        SmartMotorGroup &frontIntakeRollers;
        SmartMotorGroup &backIntakeMotors;
        SmartMotorGroup &cyclerMotors;

        OpticalSensor &colorSensor;

        ADIPneumatic &intakePneumaticsLeft;
        ADIPneumatic &intakePneumaticsRight;
        ADIPneumatic &hoodPneumatics;
    };
}