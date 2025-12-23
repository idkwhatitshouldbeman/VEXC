/**
 * @file mechanisms.hpp
 * @brief Mechanism control (intake, high scoring, pneumatics)
 */

#ifndef MECHANISMS_HPP
#define MECHANISMS_HPP

#include "api.h"

//==========================================================================
// ENUMS
//==========================================================================

/**
 * @enum IntakeState
 * @brief Current state of the intake system
 */
enum class IntakeState {
    STOPPED,        // Intake is stopped
    RUNNING,        // Intake is running normally
    STALLED,        // Stall detected
    FIXING_STALL    // Attempting to fix stall
};

/**
 * @enum RingType
 * @brief Type of ring detected by color sensor
 */
enum class RingType {
    NONE,
    RED,
    BLUE
};

/**
 * @enum HighScoringTarget
 * @brief Target positions for the high scoring mechanism
 */
enum class HighScoringTarget {
    DOWN,       // Resting position
    CAPTURE,    // Capturing rings
    WAIT,       // Waiting/holding position
    SCORE       // Scoring position
};

//==========================================================================
// INTAKE SYSTEM
//==========================================================================

namespace Intake {
    /**
     * @brief Initialize the intake system
     */
    void initialize();
    
    /**
     * @brief Set the intake state
     * @param state New state
     * @param reverse True to run in reverse
     */
    void setState(IntakeState state, bool reverse = false);
    
    /**
     * @brief Get the current intake state
     */
    IntakeState getState();
    
    /**
     * @brief Run the stall detection and handling
     * Call this periodically in the main loop
     */
    void update();
    
    /**
     * @brief Start intake (collecting)
     */
    void collect();
    
    /**
     * @brief Reverse intake (ejecting)
     */
    void eject();
    
    /**
     * @brief Stop intake
     */
    void stop();
    
    /**
     * @brief Check if intake is stalled
     */
    bool isStalled();
    
    /**
     * @brief Set the color to eject
     */
    void setEjectColor(RingType color);
    
    /**
     * @brief Get the color being ejected
     */
    RingType getEjectColor();
}

//==========================================================================
// HIGH SCORING MECHANISM
//==========================================================================

namespace HighScoring {
    /**
     * @brief Initialize the high scoring mechanism
     */
    void initialize();
    
    /**
     * @brief Set the target angle
     * @param target Target position enum
     */
    void setTarget(HighScoringTarget target);
    
    /**
     * @brief Set a specific target angle in degrees
     */
    void setTargetAngle(double angle);
    
    /**
     * @brief Get the current target angle
     */
    double getTargetAngle();
    
    /**
     * @brief Get the current actual angle
     */
    double getCurrentAngle();
    
    /**
     * @brief Update the position control
     * Call this periodically
     */
    void update();
    
    /**
     * @brief Check if mechanism is at target
     */
    bool atTarget(double tolerance = 5.0);
    
    /**
     * @brief Check if high scoring is currently running
     */
    bool isRunning();
    
    /**
     * @brief Set running state
     */
    void setRunning(bool running);
    
    /**
     * @brief Handle stall detection for high scoring
     */
    void handleStall();
}

//==========================================================================
// PNEUMATICS
//==========================================================================

namespace Pneumatics {
    /**
     * @brief Initialize all pneumatic systems
     */
    void initialize();
    
    // Mobile Goal Clamp
    void clampMogo(bool clamp);
    bool isMogoClamped();
    void toggleMogoClamp();
    
    // Ring Ejection
    void setEjection(bool eject);
    bool isEjecting();
    
    // Doinker
    void setDoinker(bool extend);
    bool isDoinkerExtended();
    void toggleDoinker();
    
    // Intake Pneumatic
    void setIntakePneumatic(bool extend);
    bool isIntakePneumaticExtended();
    void toggleIntakePneumatic();
}

//==========================================================================
// COLOR DETECTION
//==========================================================================

namespace ColorDetection {
    /**
     * @brief Initialize the color sensor
     */
    void initialize();
    
    /**
     * @brief Check the color sensor and handle ejection
     */
    void update();
    
    /**
     * @brief Get the currently detected ring type
     */
    RingType getDetectedRing();
    
    /**
     * @brief Set the color to eject
     */
    void setEjectColor(RingType color);
    
    /**
     * @brief Get the color being ejected
     */
    RingType getEjectColor();
}

//==========================================================================
// SCORING SEQUENCES
//==========================================================================

namespace Scoring {
    /**
     * @brief Scoring position types
     */
    enum class Position {
        TOP_BACK,
        TOP_FRONT,
        MID_BACK,
        MID_FRONT,
        BOTTOM
    };
    
    /**
     * @brief Execute a scoring sequence
     * @param position Scoring position
     */
    void execute(Position position);
    
    /**
     * @brief Check if a scoring sequence is in progress
     */
    bool isScoring();
    
    /**
     * @brief Cancel the current scoring sequence
     */
    void cancel();
}

#endif // MECHANISMS_HPP

