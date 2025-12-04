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

    /**
     * Represents the intake arm and claw system of the robot.
     */
    class IntakeSystem
    {
    public:
        /**
         * Creates a new instance of the intake system.
         * @param topIntakeMotors The motors controlling the top of the intake
         * @param bottomIntakeMotors The motors controlling the bottom of the intake
         * @param backIntakeMotors The motors controlling the back of the intake
         * @param cyclerMotors The motors controlling the cycler system
         * @param colorSensor The optical sensor for detecting game pieces 
         * @param intakePneumaticsLeft The left intake pneumatics
         * @param intakePneumaticsRight The right intake pneumatics
         * @param hoodPneumatics The hood pneumatics (very useful for hood shenanigans)
         */
        IntakeSystem(SmartMotorGroup &topIntakeMotors,
                     SmartMotorGroup &bottomIntakeMotors,
                     SmartMotorGroup &backIntakeMotors, 
                     SmartMotorGroup &cyclerMotors,
                     OpticalSensor &colorSensor,
                     ADIPneumatic &intakePneumaticsLeft,
                     ADIPneumatic &intakePneumaticsRight,
                     ADIPneumatic &hoodPneumatics)
            :   topIntakeMotors(topIntakeMotors), 
                bottomIntakeMotors(bottomIntakeMotors),
                backIntakeMotors(backIntakeMotors),
                cyclerMotors(cyclerMotors),
                colorSensor(colorSensor),
                intakePneumaticsLeft(intakePneumaticsLeft),
                intakePneumaticsRight(intakePneumaticsRight),
                hoodPneumatics(hoodPneumatics)

        {
            topIntakeMotors.setPosition(0);
            bottomIntakeMotors.setPosition(0);
            backIntakeMotors.setPosition(0);
            cyclerMotors.setPosition(0);

            // Max Brightness
            colorSensor.setLEDBrightness(100);
        }

        /**
         * Moves intake using speed
         * @param speed - Voltage to run intake between -1 and 1.
         */
        void defaultIntake(float speed)
        {
            speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);

            bottomIntakeMotors.moveVoltage(speed);
            backIntakeMotors.moveVoltage(speed);
            topIntakeMotors.moveVoltage(speed);
            cyclerMotors.moveVoltage(speed);
        }

        /**
         *   Outtakes game pieces to the top goal
         */
        void outtakeTop()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            backIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            bottomIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            cyclerMotors.moveVoltage(OUTTAKE_TOP_SPEED);
        }

        /**
         *   Stops outtaking game pieces
         */
        void stopOuttake()
        {
            topIntakeMotors.stop();
        }

        /**
         *   Outtakes game pieces to the middle goal
         */
        void outtakeMid()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_MIDDLE_SPEED);
            backIntakeMotors.moveVoltage(-OUTTAKE_MIDDLE_SPEED);
            bottomIntakeMotors.moveVoltage(-OUTTAKE_MIDDLE_SPEED);
            cyclerMotors.moveVoltage(-OUTTAKE_MIDDLE_SPEED);
        }

        void exitCyclerMid()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_MIDDLE_SPEED);
            backIntakeMotors.moveVoltage(-OUTTAKE_MIDDLE_SPEED);
            bottomIntakeMotors.moveVoltage(-OUTTAKE_MIDDLE_SPEED);
            cyclerMotors.moveVoltage(-EXIT_CYCLER_SPEED);
        }

        void exitCycler()
        {
            topIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            backIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            bottomIntakeMotors.moveVoltage(OUTTAKE_TOP_SPEED);
            cyclerMotors.moveVoltage(-OUTTAKE_TOP_SPEED);
        }

        void intakeExtend()
        {
            intakePneumaticsLeft.extend();
            intakePneumaticsRight.extend();
        }

        void intakeRetract()
        {
            intakePneumaticsLeft.retract();
            intakePneumaticsRight.retract();
        }

        void hoodExtend()
        {
            hoodPneumatics.extend();
        }

        void hoodRetract()
        {
            hoodPneumatics.retract();
        }

        /**
         * Sets the color sorting mode
         * @param mode - How the intake hood should sort balls
         */
        void setMode(ColorSortingMode mode) {
            sortingMode = mode;
        }

    protected:
        /**
         * Updates the intake hood to handle color sorting
         */
        void updateHoodState() {
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

        BlockColor getCurrentBlockColor() {

            // Gets the current hue of the color sensor
            double hue = colorSensor.getHue();

            // Calculate distance to the color hue (in degrees)
            double redDist = std::abs(hue - RED_BLOCK_HUE);
            double blueDist = std::abs(hue - BLUE_BLOCK_HUE);

            // Gets the block color of the closest hue
            return redDist > blueDist ?
                BlockColor::Blue :
                BlockColor::Red;
        }

    private:
        static constexpr float MAX_SPEED = 1.0;  // %
        static constexpr float MIN_SPEED = -0.6; // %

        static constexpr float OUTTAKE_TOP_SPEED = 0.8;     // %
        static constexpr float OUTTAKE_MIDDLE_SPEED = -0.5; // %
        static constexpr float EXIT_CYCLER_SPEED = 0.5; // %

        // TODO: Get color hue for block
        static constexpr float RED_BLOCK_HUE = 0.0;
        static constexpr float BLUE_BLOCK_HUE = 0.0;

        // State
        ColorSortingMode sortingMode = ColorSortingMode::DontSort;

        // Hardware
        SmartMotorGroup &topIntakeMotors;
        SmartMotorGroup &bottomIntakeMotors;
        SmartMotorGroup &backIntakeMotors;
        SmartMotorGroup &cyclerMotors;
        
        OpticalSensor &colorSensor;

        ADIPneumatic &intakePneumaticsLeft;
        ADIPneumatic &intakePneumaticsRight;
        ADIPneumatic &hoodPneumatics;
    };
}