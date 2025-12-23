/**
 * @file driver_control.hpp
 * @brief Driver control system with XABY button mechanism control
 */

#ifndef DRIVER_CONTROL_HPP
#define DRIVER_CONTROL_HPP

#include "api.h"

/**
 * @enum MechanismMode
 * @brief Current mechanism mode selected by driver
 */
enum class MechanismMode {
    NONE,           // No mode selected
    TOP_MODE,       // Top scoring selected
    MID_MODE,       // Mid scoring selected
    BOTTOM_MODE,    // Bottom scoring selected
    INTAKE_OVERRIDE // Manual intake control
};

namespace DriverControl {
    /**
     * @brief Initialize driver control systems
     */
    void initialize();
    
    /**
     * @brief Run the driver control loop (call continuously)
     * Handles all driver inputs including:
     * - Tank drive control
     * - XABY button mechanism selection
     * - Trigger buttons for execution
     * - Pneumatic controls
     */
    void update();
    
    /**
     * @brief Start the driver control task
     */
    void startTask();
    
    /**
     * @brief Stop the driver control task
     */
    void stopTask();
    
    /**
     * @brief Check if driver control task is running
     */
    bool isRunning();
    
    //==========================================================================
    // DRIVE CONTROL
    //==========================================================================
    
    /**
     * @brief Set drive motor velocities based on controller input
     */
    void setDriveVelocities();
    
    /**
     * @brief Toggle slow drive mode
     */
    void toggleSlowDrive();
    
    /**
     * @brief Check if slow drive is enabled
     */
    bool isSlowDriveEnabled();
    
    /**
     * @brief Stop all drive motors
     */
    void stopDrive();
    
    //==========================================================================
    // MECHANISM CONTROL
    //==========================================================================
    
    /**
     * @brief Get the current mechanism mode
     */
    MechanismMode getCurrentMode();
    
    /**
     * @brief Set the current mechanism mode
     */
    void setCurrentMode(MechanismMode mode);
    
    /**
     * @brief Handle XABY button presses for mode selection
     */
    void handleModeSelection();
    
    /**
     * @brief Handle trigger buttons for action execution
     */
    void handleActionButtons();
    
    /**
     * @brief Handle intake toggle buttons
     */
    void handleIntakeButtons();
    
    /**
     * @brief Handle high scoring motor buttons
     */
    void handleHighScoringButtons();
    
    /**
     * @brief Handle pneumatic control buttons
     */
    void handlePneumaticButtons();
    
    //==========================================================================
    // DISPLAY
    //==========================================================================
    
    /**
     * @brief Update the controller screen display
     */
    void updateControllerDisplay();
    
    /**
     * @brief Display a message on the controller
     */
    void displayMessage(const char* message);
}

#endif // DRIVER_CONTROL_HPP

