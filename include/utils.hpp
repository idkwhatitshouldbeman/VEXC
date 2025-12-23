/**
 * @file utils.hpp
 * @brief Utility functions and helpers
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include "api.h"
#include <string>
#include <vector>
#include <cmath>

namespace Utils {
    //==========================================================================
    // MATH HELPERS
    //==========================================================================
    
    /**
     * @brief Clamp a value between min and max
     */
    template<typename T>
    T clamp(T value, T min_val, T max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }
    
    /**
     * @brief Normalize angle to range [-PI, PI]
     */
    inline double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * @brief Convert degrees to radians
     */
    inline double degToRad(double degrees) {
        return degrees * M_PI / 180.0;
    }
    
    /**
     * @brief Convert radians to degrees
     */
    inline double radToDeg(double radians) {
        return radians * 180.0 / M_PI;
    }
    
    /**
     * @brief Calculate distance between two points
     */
    inline double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    /**
     * @brief Calculate the sign of a number
     */
    template<typename T>
    int sign(T val) {
        return (T(0) < val) - (val < T(0));
    }
    
    /**
     * @brief Linear interpolation
     */
    inline double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }
    
    /**
     * @brief Apply deadband to a value
     */
    inline double applyDeadband(double value, double deadband) {
        if (std::abs(value) < deadband) return 0.0;
        return value;
    }
    
    /**
     * @brief Average of an array
     */
    inline double average(const std::vector<double>& values) {
        if (values.empty()) return 0.0;
        double sum = 0.0;
        for (double v : values) sum += v;
        return sum / values.size();
    }
    
    /**
     * @brief Cubic scaling for joystick input
     */
    inline double cubicScale(double input, double maxOutput = 100.0) {
        double normalized = input / 100.0;
        double scaled = normalized * normalized * normalized;
        return scaled * maxOutput;
    }
    
    //==========================================================================
    // STRING HELPERS
    //==========================================================================
    
    /**
     * @brief Split a string by delimiter
     */
    std::vector<std::string> split(const std::string& str, char delimiter);
    
    /**
     * @brief Trim whitespace from string
     */
    std::string trim(const std::string& str);
    
    /**
     * @brief Parse a double from JSON-like string
     */
    double parseJsonDouble(const std::string& json, const std::string& key);
    
    //==========================================================================
    // FILE HELPERS
    //==========================================================================
    
    /**
     * @brief Check if a file exists on SD card
     */
    bool fileExists(const char* filename);
    
    /**
     * @brief Read entire file to string
     */
    std::string readFile(const char* filename);
    
    /**
     * @brief Write string to file
     */
    bool writeFile(const char* filename, const std::string& content);
    
    /**
     * @brief Append string to file
     */
    bool appendFile(const char* filename, const std::string& content);
    
    //==========================================================================
    // CONTROLLER HELPERS
    //==========================================================================
    
    /**
     * @brief Wait for specific button press
     */
    void waitForButton(pros::Controller& controller, pros::controller_digital_e_t button);
    
    /**
     * @brief Wait for any button press
     */
    void waitForAnyButton(pros::Controller& controller);
    
    /**
     * @brief Rumble the controller
     */
    void rumble(pros::Controller& controller, const char* pattern);
}

#endif // UTILS_HPP

