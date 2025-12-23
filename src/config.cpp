/**
 * @file config.cpp
 * @brief Configuration implementation and calibration data management
 */

#include "config.hpp"
#include "utils.hpp"
#include <cstdio>

namespace Config {
    //==========================================================================
    // RUNTIME CALIBRATED VALUES (initialized with defaults)
    //==========================================================================
    
    double forward_wheel_offset = DEFAULT_FORWARD_WHEEL_OFFSET;
    double sideways_wheel_offset = DEFAULT_SIDEWAYS_WHEEL_OFFSET;
    double actual_drivetrain_diameter = DRIVETRAIN_WHEEL_DIAMETER;
    double actual_odom_diameter = ODOM_WHEEL_DIAMETER;
    double track_width = DEFAULT_TRACK_WIDTH;
    double lookahead_distance = DEFAULT_LOOKAHEAD_DISTANCE;
    
    // PID values
    double pid_linear_p = LINEAR_KP;
    double pid_linear_i = LINEAR_KI;
    double pid_linear_d = LINEAR_KD;
    
    double pid_angular_p = ANGULAR_KP;
    double pid_angular_i = ANGULAR_KI;
    double pid_angular_d = ANGULAR_KD;
    
    double pid_turn_p = TURN_KP;
    double pid_turn_i = TURN_KI;
    double pid_turn_d = TURN_KD;
    
    void initDefaults() {
        forward_wheel_offset = DEFAULT_FORWARD_WHEEL_OFFSET;
        sideways_wheel_offset = DEFAULT_SIDEWAYS_WHEEL_OFFSET;
        actual_drivetrain_diameter = DRIVETRAIN_WHEEL_DIAMETER;
        actual_odom_diameter = ODOM_WHEEL_DIAMETER;
        track_width = DEFAULT_TRACK_WIDTH;
        lookahead_distance = DEFAULT_LOOKAHEAD_DISTANCE;
        
        pid_linear_p = LINEAR_KP;
        pid_linear_i = LINEAR_KI;
        pid_linear_d = LINEAR_KD;
        
        pid_angular_p = ANGULAR_KP;
        pid_angular_i = ANGULAR_KI;
        pid_angular_d = ANGULAR_KD;
        
        pid_turn_p = TURN_KP;
        pid_turn_i = TURN_KI;
        pid_turn_d = TURN_KD;
    }
    
    bool loadCalibration() {
        if (!Utils::fileExists(CALIBRATION_FILE)) {
            printf("Calibration file not found, using defaults\n");
            return false;
        }
        
        std::string content = Utils::readFile(CALIBRATION_FILE);
        if (content.empty()) {
            printf("Failed to read calibration file\n");
            return false;
        }
        
        // Parse JSON values
        forward_wheel_offset = Utils::parseJsonDouble(content, "forward_wheel_offset");
        sideways_wheel_offset = Utils::parseJsonDouble(content, "sideways_wheel_offset");
        actual_drivetrain_diameter = Utils::parseJsonDouble(content, "actual_drivetrain_diameter");
        actual_odom_diameter = Utils::parseJsonDouble(content, "actual_odom_diameter");
        track_width = Utils::parseJsonDouble(content, "track_width");
        lookahead_distance = Utils::parseJsonDouble(content, "lookahead_distance");
        
        pid_linear_p = Utils::parseJsonDouble(content, "pid_linear_p");
        pid_linear_i = Utils::parseJsonDouble(content, "pid_linear_i");
        pid_linear_d = Utils::parseJsonDouble(content, "pid_linear_d");
        
        pid_angular_p = Utils::parseJsonDouble(content, "pid_angular_p");
        pid_angular_i = Utils::parseJsonDouble(content, "pid_angular_i");
        pid_angular_d = Utils::parseJsonDouble(content, "pid_angular_d");
        
        pid_turn_p = Utils::parseJsonDouble(content, "pid_turn_p");
        pid_turn_i = Utils::parseJsonDouble(content, "pid_turn_i");
        pid_turn_d = Utils::parseJsonDouble(content, "pid_turn_d");
        
        printf("Calibration loaded successfully\n");
        return true;
    }
    
    bool saveCalibration() {
        char buffer[2048];
        snprintf(buffer, sizeof(buffer),
            "{\n"
            "  \"forward_wheel_offset\": %.6f,\n"
            "  \"sideways_wheel_offset\": %.6f,\n"
            "  \"actual_drivetrain_diameter\": %.6f,\n"
            "  \"actual_odom_diameter\": %.6f,\n"
            "  \"track_width\": %.6f,\n"
            "  \"lookahead_distance\": %.6f,\n"
            "  \"pid_linear_p\": %.6f,\n"
            "  \"pid_linear_i\": %.6f,\n"
            "  \"pid_linear_d\": %.6f,\n"
            "  \"pid_angular_p\": %.6f,\n"
            "  \"pid_angular_i\": %.6f,\n"
            "  \"pid_angular_d\": %.6f,\n"
            "  \"pid_turn_p\": %.6f,\n"
            "  \"pid_turn_i\": %.6f,\n"
            "  \"pid_turn_d\": %.6f\n"
            "}\n",
            forward_wheel_offset,
            sideways_wheel_offset,
            actual_drivetrain_diameter,
            actual_odom_diameter,
            track_width,
            lookahead_distance,
            pid_linear_p,
            pid_linear_i,
            pid_linear_d,
            pid_angular_p,
            pid_angular_i,
            pid_angular_d,
            pid_turn_p,
            pid_turn_i,
            pid_turn_d
        );
        
        if (Utils::writeFile(CALIBRATION_FILE, buffer)) {
            printf("Calibration saved successfully\n");
            return true;
        }
        
        printf("Failed to save calibration\n");
        return false;
    }
}

