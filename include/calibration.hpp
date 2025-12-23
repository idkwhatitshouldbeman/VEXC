/**
 * @file calibration.hpp
 * @brief Calibration system for auto-tuning robot parameters
 */

#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include "api.h"

namespace Calibration {
    /**
     * @brief Check if calibration mode should be entered
     * @return true if UP + X buttons are held during startup
     */
    bool shouldEnterCalibrationMode();
    
    /**
     * @brief Run the full calibration sequence
     */
    void runCalibrationMode();
    
    /**
     * @brief Load calibration data from SD card
     * @return true if loaded successfully
     */
    bool loadCalibrationData();
    
    /**
     * @brief Save calibration data to SD card
     * @return true if saved successfully
     */
    bool saveCalibrationData();
    
    //==========================================================================
    // INDIVIDUAL CALIBRATION STEPS
    //==========================================================================
    
    /**
     * @brief Calibrate tracking wheel positions
     * Determines forward and sideways offsets from center of rotation
     */
    void calibrateTrackingWheelPositions();
    
    /**
     * @brief Calibrate wheel diameters
     * Verifies actual wheel diameter vs nominal
     */
    void calibrateWheelDiameters();
    
    /**
     * @brief Calibrate track width
     * Measures actual distance between wheels
     */
    void calibrateTrackWidth();
    
    /**
     * @brief Characterize robot motion
     * Tests slip, acceleration, deceleration
     */
    void characterizeMotion();
    
    /**
     * @brief Calibrate distance sensors
     * Creates correction table for sensors
     */
    void calibrateDistanceSensors();
    
    /**
     * @brief Auto-tune PID controllers
     * Uses Ziegler-Nichols method
     */
    void autoTunePID();
    
    //==========================================================================
    // HELPER FUNCTIONS
    //==========================================================================
    
    /**
     * @brief Wait for button press with display message
     */
    void waitForButtonWithMessage(pros::controller_digital_e_t button, const char* message);
    
    /**
     * @brief Read a value from the terminal
     */
    double readFromTerminal(const char* prompt);
    
    /**
     * @brief Display progress on brain screen
     */
    void displayProgress(int step, int totalSteps, const char* description);
    
    /**
     * @brief Spin the robot in place for calibration
     * @param degrees Number of degrees to spin
     * @param velocity Velocity in percent
     */
    void spinInPlace(double degrees, double velocity = 30.0);
    
    /**
     * @brief Drive forward for calibration
     * @param rotations Number of wheel rotations
     * @param velocity Velocity in percent
     */
    void driveForward(double rotations, double velocity = 50.0);
    
    /**
     * @brief Detect oscillation for PID tuning
     * @return true if sustained oscillation detected
     */
    bool detectOscillation();
    
    /**
     * @brief Measure the period of oscillation
     * @return Period in seconds
     */
    double measureOscillationPeriod();
    
    //==========================================================================
    // CALIBRATION STATE
    //==========================================================================
    
    /**
     * @brief Check if calibration is currently running
     */
    bool isCalibrating();
    
    /**
     * @brief Check if calibration data has been loaded
     */
    bool isCalibrationLoaded();
}

#endif // CALIBRATION_HPP

