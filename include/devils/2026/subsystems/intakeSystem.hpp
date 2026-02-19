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
        IntakeSystem(SmartMotorGroup& frontBottomIntakeMotors,
                     SmartMotorGroup& frontTopIntakeMotors,
                     SmartMotorGroup& frontIntakeRollers,
                     SmartMotorGroup& backIntakeMotors,
                     SmartMotorGroup& cyclerMotors,
                     OpticalSensor& colorSensor,
                     ADIPneumatic& intakePneumaticsLeft,
                     ADIPneumatic& intakePneumaticsRight,
                     ADIPneumatic& hoodPneumatics)
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
            colorSensor.setLEDBrightness(1);
        }

        /**
         * Moves intake using speed
         * @param speed - Voltage to run intake between -1 and 1.
         */
        void runIntake(float speed)
        {
            // Check for stall
            if (checkForStall())
                return;

            // Calculate the speeds for each motor
            speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);

            float frontBottomIntakeSpeed = speed * FRONT_BOTTOM_INTAKE_SPEED;
            float frontTopIntakeSpeed = speed * FRONT_TOP_INTAKE_SPEED;
            float frontIntakeRollerSpeed = speed * ROLLER_SPEED;
            float backIntakeSpeed = speed * BACK_INTAKE_SPEED;
            float cyclerSpeed = speed * CYCLER_SPEED;

            switch (intakeMode)
            {
            case Cycler:
                frontBottomIntakeMotors.move(frontBottomIntakeSpeed);
                frontTopIntakeMotors.move(frontTopIntakeSpeed);
                frontIntakeRollers.move(frontIntakeRollerSpeed);
                // Absolute value is used to ensure the back intake *always* moves in the same direction
                backIntakeMotors.move(std::fabs(backIntakeSpeed));
                cyclerMotors.move(cyclerSpeed);

                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(false);
                break;

            case SideGoal:
                frontBottomIntakeMotors.move(frontBottomIntakeSpeed);
                frontTopIntakeMotors.move(frontTopIntakeSpeed);
                frontIntakeRollers.move(frontIntakeRollerSpeed);
                backIntakeMotors.move(backIntakeSpeed);
                cyclerMotors.move(-cyclerSpeed);
                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(true);
                break;

            case MidTop:
                frontBottomIntakeMotors.move(-frontBottomIntakeSpeed);
                frontTopIntakeMotors.move(frontTopIntakeSpeed);
                frontIntakeRollers.move(frontIntakeRollerSpeed);
                backIntakeMotors.move(backIntakeSpeed);
                cyclerMotors.move(-cyclerSpeed);
                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(true);
                break;

            case MidBottom:
                frontBottomIntakeMotors.move(frontBottomIntakeSpeed);
                frontTopIntakeMotors.move(-frontTopIntakeSpeed);
                frontIntakeRollers.move(frontIntakeRollerSpeed);
                backIntakeMotors.move(backIntakeSpeed);
                cyclerMotors.move(-cyclerSpeed);
                if (isHoodAutomatic)
                    hoodPneumatics.setExtended(true);
                break;
            }
        }

        /**
         * Extends/retracts the intake arms
         * @param extended - Whether the arms should be extended
         */
        void setArmsExtended(const bool extended) const
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
         * Checks for a stall and handles it if detected.
         * @return True if a stall is being handled, false otherwise.
         */
        bool checkForStall()
        {
            // Check if we are reversing because of a stall
            bool isReversingBecauseStall = reverseBecauseStallTimer.getIsRunning();
            if (isReversingBecauseStall)
            {
                frontBottomIntakeMotors.move(STALL_SPEED);
                frontTopIntakeMotors.move(STALL_SPEED);
                frontIntakeRollers.move(STALL_SPEED);
                backIntakeMotors.move(STALL_SPEED);
                cyclerMotors.move(STALL_SPEED);
                return true;
            }

            // Search for stall condition
            bool isFrontBottomStalled = frontBottomIntakeMotors.getCurrent() > STALL_CURRENT;
            bool isFrontTopStalled = frontTopIntakeMotors.getCurrent() > STALL_CURRENT;
            bool isStalled = isFrontBottomStalled || isFrontTopStalled;

            if (isStalled)
            {
                // Once the timer is finished, start reversing
                if (checkForStallTimer.getIsFinished())
                    reverseBecauseStallTimer.start();

                // Start the min stall timer.
                // This prevents false positives from quick current spikes.
                if (!checkForStallTimer.getIsFinished())
                    checkForStallTimer.start();
            }
            else
            {
                checkForStallTimer.stop();
            }

            return false;
        }

        /**
         * Updates the intake hood to handle color sorting
         */
        void setHoodExtendedBasedOnColorSort()
        {
            if (sortingMode == DontSort)
                return;

            const BlockColor currentBlockColor = getCurrentBlockColor();

            // Check if we can score either color
            const bool canScoreBlue = currentBlockColor == Blue &&
                sortingMode == ScoreBlueOnly;

            const bool canScoreRed = currentBlockColor == Blue &&
                sortingMode == ScoreRedOnly;

            // If we can score, retract the hood
            // Otherwise, extend the hood
            const bool isCorrectSorting = canScoreBlue || canScoreRed;
            hoodPneumatics.setExtended(!isCorrectSorting);
        }

        /**
         * Gets the current color of the block in the intake.
         * @return The current color of the block in the intake.
         */
        BlockColor getCurrentBlockColor() const
        {
            // Gets the current hue of the color sensor
            const float hue = colorSensor.getHue();

            // Calculate distance to the color hue (in degrees)
            const float redDist = std::abs(hue - RED_BLOCK_HUE);
            const float blueDist = std::abs(hue - BLUE_BLOCK_HUE);

            // Gets the block color of the closest hue
            return redDist > blueDist ? Blue : Red;
        }

    private:
        //     SPEED OPTIONS

        static constexpr float MAX_SPEED = 1.0; // %
        static constexpr float MIN_SPEED = -1.0; // %

        static constexpr float FRONT_BOTTOM_INTAKE_SPEED = 0.7; // %
        static constexpr float FRONT_TOP_INTAKE_SPEED = 0.7; // %
        static constexpr float ROLLER_SPEED = 1.0; // %
        static constexpr float BACK_INTAKE_SPEED = 0.6; // %
        static constexpr float CYCLER_SPEED = 1.0; // %

        //      COLOR SENSOR OPTIONS

        // TODO: Get color hue for block
        static constexpr float RED_BLOCK_HUE = 0.0;
        static constexpr float BLUE_BLOCK_HUE = 0.0;

        //     STALL OPTIONS

        /// @brief The current threshold to detect a stall (in mA).
        static constexpr float STALL_CURRENT = 1600;

        /// @brief The minimum duration a stall must occur.
        static constexpr int32_t STALL_MIN_DURATION = 600;

        /// @brief The duration to run backwards when stall is detected.
        static constexpr int32_t STALL_REVERSE_DURATION = 300;

        /// @brief The speed to run while stalled.
        static constexpr float STALL_SPEED = -1.0;

        // State
        ColorSortingMode sortingMode = DontSort;
        IntakeMode intakeMode = Cycler;
        bool isHoodAutomatic = true;
        Timer checkForStallTimer = Timer(STALL_MIN_DURATION);
        Timer reverseBecauseStallTimer = Timer(STALL_REVERSE_DURATION);

        // Hardware
        SmartMotorGroup& frontBottomIntakeMotors;
        SmartMotorGroup& frontTopIntakeMotors;
        SmartMotorGroup& frontIntakeRollers;
        SmartMotorGroup& backIntakeMotors;
        SmartMotorGroup& cyclerMotors;

        OpticalSensor& colorSensor;

        ADIPneumatic& intakePneumaticsLeft;
        ADIPneumatic& intakePneumaticsRight;
        ADIPneumatic& hoodPneumatics;
    };
}
