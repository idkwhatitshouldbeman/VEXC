/**
 * @file autonomous.hpp
 * @brief Autonomous routines and instruction system
 */

#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "api.h"
#include "pure_pursuit.hpp"
#include "mechanisms.hpp"
#include <vector>
#include <string>

//==========================================================================
// INSTRUCTION SYSTEM
//==========================================================================

/**
 * @enum InstructionType
 * @brief Types of instructions in the autonomous script
 */
enum class InstructionType {
    INIT,           // Set starting position: INIT x y heading_deg
    PATH,           // Follow a path: PATH filename.csv
    WAIT_PATH,      // Wait for current path to complete
    SCORE,          // Execute scoring: SCORE position
    INTAKE,         // Control intake: INTAKE state
    PTO,            // Control PTO: PTO state
    WAIT,           // Wait for time: WAIT ms
    TURN_TO,        // Turn to heading: TURN_TO heading_deg
    DRIVE_TO,       // Drive to point: DRIVE_TO x y
    MOGO,           // Control mogo clamp: MOGO on/off
    DOINKER,        // Control doinker: DOINKER on/off
    HIGH_SCORE,     // Set high scoring position: HIGH_SCORE position
    SET_VELOCITY,   // Set forward velocity: SET_VELOCITY vel
    SET_EJECT,      // Set eject color: SET_EJECT color
    PRINT,          // Print debug message: PRINT message
    END             // End of program
};

/**
 * @struct Instruction
 * @brief A single instruction with parameters
 */
struct Instruction {
    InstructionType type;
    std::vector<std::string> params;
    
    Instruction() : type(InstructionType::END) {}
    Instruction(InstructionType t) : type(t) {}
};

//==========================================================================
// AUTONOMOUS ROUTINES
//==========================================================================

namespace Autonomous {
    /**
     * @brief Available autonomous routine slots
     */
    enum class Slot {
        RED_LEFT = 1,
        RED_RIGHT = 2,
        BLUE_LEFT = 3,
        BLUE_RIGHT = 4,
        SKILLS = 5,
        TEST = 6
    };
    
    /**
     * @brief Initialize the autonomous system
     */
    void initialize();
    
    /**
     * @brief Set the autonomous slot to run
     */
    void setSlot(Slot slot);
    
    /**
     * @brief Get the current autonomous slot
     */
    Slot getSlot();
    
    /**
     * @brief Run the selected autonomous routine
     */
    void run();
    
    /**
     * @brief Run a specific autonomous slot
     */
    void run(Slot slot);
    
    //==========================================================================
    // INSTRUCTION SYSTEM
    //==========================================================================
    
    /**
     * @brief Load instructions from a file
     * @param filename Path to instruction file on SD card
     * @return Vector of instructions
     */
    std::vector<Instruction> loadInstructions(const std::string& filename);
    
    /**
     * @brief Execute a list of instructions
     * @param instructions Instructions to execute
     */
    void executeInstructions(const std::vector<Instruction>& instructions);
    
    /**
     * @brief Execute a single instruction
     * @param inst Instruction to execute
     */
    void executeInstruction(const Instruction& inst);
    
    //==========================================================================
    // PRE-DEFINED ROUTINES
    //==========================================================================
    
    /**
     * @brief Red left side autonomous (AWP)
     */
    void redLeft();
    
    /**
     * @brief Red right side autonomous (AWP)
     */
    void redRight();
    
    /**
     * @brief Blue left side autonomous (AWP)
     */
    void blueLeft();
    
    /**
     * @brief Blue right side autonomous (AWP)
     */
    void blueRight();
    
    /**
     * @brief Skills autonomous
     */
    void skills();
    
    /**
     * @brief Test autonomous
     */
    void test();
    
    //==========================================================================
    // BASIC MOVEMENT FUNCTIONS
    //==========================================================================
    
    /**
     * @brief Turn to an absolute heading
     * @param targetHeading Target heading in radians
     * @param timeout Timeout in milliseconds (0 = no timeout)
     */
    void turnToHeading(double targetHeading, int timeout = 0);
    
    /**
     * @brief Turn to an absolute heading in degrees
     */
    void turnToHeadingDeg(double targetDegrees, int timeout = 0);
    
    /**
     * @brief Drive to a specific point
     * @param x Target X coordinate
     * @param y Target Y coordinate
     * @param timeout Timeout in milliseconds
     */
    void driveToPoint(double x, double y, int timeout = 0);
    
    /**
     * @brief Drive forward/backward for a distance
     * @param distance Distance in inches (negative for reverse)
     * @param velocity Velocity in percent
     */
    void driveDistance(double distance, double velocity = 50.0);
    
    /**
     * @brief Wait for path following to complete
     * @param timeout Timeout in milliseconds (0 = no timeout)
     */
    void waitForPath(int timeout = 0);
    
    //==========================================================================
    // STATE
    //==========================================================================
    
    /**
     * @brief Check if autonomous is currently running
     */
    bool isRunning();
    
    /**
     * @brief Stop autonomous execution
     */
    void stop();
}

#endif // AUTONOMOUS_HPP

