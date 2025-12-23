/**
 * @file telemetry.hpp
 * @brief Telemetry, logging, and display systems
 */

#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include "api.h"
#include <string>

namespace Telemetry {
    /**
     * @brief Initialize the telemetry system
     */
    void initialize();
    
    /**
     * @brief Start the telemetry update task
     */
    void startTask();
    
    /**
     * @brief Stop the telemetry update task
     */
    void stopTask();
    
    /**
     * @brief Check if telemetry task is running
     */
    bool isRunning();
    
    //==========================================================================
    // TERMINAL OUTPUT
    //==========================================================================
    
    /**
     * @brief Print robot status to terminal
     */
    void printStatus();
    
    /**
     * @brief Print a formatted message to terminal
     */
    void print(const char* format, ...);
    
    /**
     * @brief Print a debug message (only if debug enabled)
     */
    void debug(const char* format, ...);
    
    /**
     * @brief Enable/disable debug output
     */
    void setDebugEnabled(bool enabled);
    
    /**
     * @brief Check if debug is enabled
     */
    bool isDebugEnabled();
    
    //==========================================================================
    // BRAIN SCREEN DISPLAY
    //==========================================================================
    
    /**
     * @brief Update the brain screen display
     */
    void updateBrainScreen();
    
    /**
     * @brief Clear the brain screen
     */
    void clearBrainScreen();
    
    /**
     * @brief Display a message on the brain screen
     */
    void displayMessage(int line, const char* format, ...);
    
    /**
     * @brief Display an error on the brain screen
     */
    void displayError(const char* message);
    
    /**
     * @brief Display the current robot position
     */
    void displayPosition();
    
    /**
     * @brief Display the current mode
     */
    void displayMode();
    
    //==========================================================================
    // CONTROLLER DISPLAY
    //==========================================================================
    
    /**
     * @brief Update the controller screen
     */
    void updateControllerScreen();
    
    /**
     * @brief Set controller screen line
     */
    void setControllerLine(int line, const char* text);
    
    /**
     * @brief Clear the controller screen
     */
    void clearControllerScreen();
    
    //==========================================================================
    // SD CARD LOGGING
    //==========================================================================
    
    /**
     * @brief Start logging to SD card
     */
    void startLogging();
    
    /**
     * @brief Stop logging to SD card
     */
    void stopLogging();
    
    /**
     * @brief Check if logging is active
     */
    bool isLogging();
    
    /**
     * @brief Log current state to SD card
     */
    void logState();
    
    /**
     * @brief Log a custom message
     */
    void log(const char* format, ...);
    
    /**
     * @brief Log position data
     */
    void logPosition();
    
    /**
     * @brief Get the log filename
     */
    std::string getLogFilename();
    
    //==========================================================================
    // STATISTICS
    //==========================================================================
    
    /**
     * @brief Get the battery voltage
     */
    double getBatteryVoltage();
    
    /**
     * @brief Get the battery current
     */
    double getBatteryCurrent();
    
    /**
     * @brief Get the battery capacity percentage
     */
    int getBatteryCapacity();
    
    /**
     * @brief Get motor temperature
     */
    double getMotorTemperature(int port);
    
    /**
     * @brief Check system health
     * @return true if all systems are healthy
     */
    bool checkSystemHealth();
    
    /**
     * @brief Get uptime in milliseconds
     */
    uint32_t getUptime();
}

#endif // TELEMETRY_HPP

