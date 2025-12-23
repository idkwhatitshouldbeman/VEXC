/**
 * @file config.hpp
 * @brief Hardware configuration and constants
 * 
 * All configurable parameters for the robot.
 * Port assignments from reference: https://github.com/arvindkandhare/dt_working
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "api.h"

namespace Config {
    //==========================================================================
    // MOTOR PORTS
    //==========================================================================
    
    // Left Drivetrain Motors (600 RPM blue cartridge, reversed)
    constexpr int LEFT_MOTOR_A_PORT = 2;
    constexpr int LEFT_MOTOR_B_PORT = 3;
    constexpr int LEFT_MOTOR_C_PORT = 4;
    
    // Right Drivetrain Motors (600 RPM blue cartridge, not reversed)
    constexpr int RIGHT_MOTOR_A_PORT = 5;
    constexpr int RIGHT_MOTOR_B_PORT = 12;
    constexpr int RIGHT_MOTOR_C_PORT = 17;
    
    // Mechanism Motors
    constexpr int HIGH_SCORING_MOTOR_PORT = 20;
    constexpr int INTAKE_LOWER_PORT = 21;
    constexpr int INTAKE_UPPER_PORT = 13;
    
    //==========================================================================
    // SENSOR PORTS
    //==========================================================================
    
    // IMU (Inertial Measurement Unit)
    constexpr int IMU_PORT = 9;
    
    // Rotational Sensors (Tracking Wheels)
    constexpr int VERTICAL_ENCODER_PORT = 7;    // Forward/back tracking
    constexpr int HORIZONTAL_ENCODER_PORT = 6;  // Sideways tracking
    
    // High Scoring Position Sensor
    constexpr int HIGH_SCORING_ROTATION_PORT = 19;
    
    // Optical/Color Sensor
    constexpr int COLOR_SENSOR_PORT = 15;
    
    //==========================================================================
    // ADI (3-WIRE) PORTS
    //==========================================================================
    
    // Pneumatic Solenoids
    constexpr char MOGO_CLAMP_PORT = 'F';        // Mobile goal clamp
    constexpr char EJECTION_PORT = 'A';          // Ring ejection
    constexpr char DOINKER_PORT = 'H';           // Doinker mechanism
    constexpr char INTAKE_PNEUMATIC_PORT = 'D';  // Intake control
    
    //==========================================================================
    // DRIVETRAIN CONFIGURATION
    //==========================================================================
    
    // Wheel specifications
    constexpr double DRIVETRAIN_WHEEL_DIAMETER = 3.25;  // inches
    constexpr double DRIVETRAIN_GEAR_RATIO = 600.0;     // RPM (blue cartridge)
    constexpr int MOTORS_PER_SIDE = 3;
    
    // Track width (distance between left and right wheels)
    constexpr double DEFAULT_TRACK_WIDTH = 12.5;        // inches (default, calibrated value loaded from SD)
    
    //==========================================================================
    // ODOMETRY CONFIGURATION
    //==========================================================================
    
    // Tracking wheel specifications
    constexpr double ODOM_WHEEL_DIAMETER = 2.0;         // inches
    constexpr double ODOM_GEAR_RATIO = 1.0;             // Direct drive
    
    // Default tracking wheel offsets (calibrated values loaded from SD)
    constexpr double DEFAULT_FORWARD_WHEEL_OFFSET = 0.0;    // inches from center of rotation
    constexpr double DEFAULT_SIDEWAYS_WHEEL_OFFSET = 0.0;   // inches from center of rotation
    
    // Conversion factor for encoders (degrees to inches)
    constexpr double ODOM_WHEEL_CIRCUMFERENCE = ODOM_WHEEL_DIAMETER * M_PI;
    constexpr double DEGREES_TO_INCHES = ODOM_WHEEL_CIRCUMFERENCE / 360.0;
    
    // Feet to internal units conversion (from reference code)
    constexpr double FEET_TO_UNIT = 2.5;
    
    //==========================================================================
    // PURE PURSUIT CONFIGURATION
    //==========================================================================
    
    constexpr double DEFAULT_LOOKAHEAD_DISTANCE = 50.0;   // units (from reference)
    constexpr double DEFAULT_PATH_TOLERANCE = 6.0;        // units
    constexpr double MIN_LOOKAHEAD = 6.0;                 // minimum lookahead distance
    constexpr double MAX_LOOKAHEAD = 24.0;                // maximum lookahead distance
    
    // Speed limits
    constexpr double MAX_VELOCITY = 100.0;                // percent
    constexpr double DEFAULT_FORWARD_VELOCITY = 40.0;     // percent
    constexpr double DEFAULT_TURN_VELOCITY_K = 40.0;      // turning gain
    
    //==========================================================================
    // HIGH SCORING ANGLES (from reference)
    //==========================================================================
    
    constexpr double HIGH_SCORE_ANGLE_DOWN = 0.0;
    constexpr double HIGH_SCORE_ANGLE_CAPTURE = -60.0;
    constexpr double HIGH_SCORE_ANGLE_WAIT = -200.0;
    constexpr double HIGH_SCORE_ANGLE_SCORE = -430.0;
    
    constexpr int MAX_CAPTURE_POSITION_COUNT = 51;
    
    //==========================================================================
    // INTAKE CONFIGURATION
    //==========================================================================
    
    constexpr int INTAKE_LOWER_VELOCITY = 90;   // percent
    constexpr int INTAKE_UPPER_VELOCITY = 80;   // percent
    constexpr int STALL_THRESHOLD = 0;          // minimum velocity to detect stall
    constexpr int STALL_COUNT = 20;             // cycles before stall is confirmed
    constexpr int RETRY_LIMIT = 15;             // attempts to fix stall
    constexpr int EJECT_LIMIT = 20;             // eject counter limit
    
    //==========================================================================
    // COLOR DETECTION
    //==========================================================================
    
    constexpr int MIN_REJECT_SIZE = 5000;       // minimum size for color rejection
    constexpr int BRIGHTNESS_THRESHOLD = 0;     // optical sensor brightness threshold
    
    //==========================================================================
    // TIMING CONSTANTS
    //==========================================================================
    
    constexpr int ODOMETRY_LOOP_DELAY = 10;     // ms (100Hz)
    constexpr int DRIVER_LOOP_DELAY = 20;       // ms (50Hz)
    constexpr int AUTONOMOUS_LOOP_DELAY = 20;   // ms (50Hz)
    constexpr int TELEMETRY_LOOP_DELAY = 100;   // ms (10Hz)
    
    //==========================================================================
    // PID DEFAULT VALUES (will be calibrated)
    //==========================================================================
    
    // Linear PID (driving forward/backward)
    constexpr double LINEAR_KP = 1.0;
    constexpr double LINEAR_KI = 0.0;
    constexpr double LINEAR_KD = 0.1;
    
    // Angular PID (heading correction during drive)
    constexpr double ANGULAR_KP = 2.0;
    constexpr double ANGULAR_KI = 0.0;
    constexpr double ANGULAR_KD = 0.2;
    
    // Turn PID (point turns)
    constexpr double TURN_KP = 2.5;
    constexpr double TURN_KI = 0.0;
    constexpr double TURN_KD = 0.3;
    
    //==========================================================================
    // CALIBRATION FILE PATHS
    //==========================================================================
    
    constexpr const char* CALIBRATION_FILE = "/usd/calibration.json";
    constexpr const char* LOG_FILE = "/usd/match_log.csv";
    
    //==========================================================================
    // RUNTIME CALIBRATED VALUES (loaded from SD card)
    //==========================================================================
    
    // These are loaded from calibration file at startup
    extern double forward_wheel_offset;
    extern double sideways_wheel_offset;
    extern double actual_drivetrain_diameter;
    extern double actual_odom_diameter;
    extern double track_width;
    extern double lookahead_distance;
    
    // PID values (loaded from calibration)
    extern double pid_linear_p, pid_linear_i, pid_linear_d;
    extern double pid_angular_p, pid_angular_i, pid_angular_d;
    extern double pid_turn_p, pid_turn_i, pid_turn_d;
    
    /**
     * @brief Initialize calibrated values with defaults
     */
    void initDefaults();
    
    /**
     * @brief Load calibration data from SD card
     * @return true if loaded successfully, false otherwise
     */
    bool loadCalibration();
    
    /**
     * @brief Save calibration data to SD card
     * @return true if saved successfully, false otherwise
     */
    bool saveCalibration();
}

#endif // CONFIG_HPP

