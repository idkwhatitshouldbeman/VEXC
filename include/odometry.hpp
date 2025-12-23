/**
 * @file odometry.hpp
 * @brief Odometry system for position tracking
 * 
 * Uses two tracking wheels + IMU for accurate position tracking
 */

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "api.h"
#include <atomic>

/**
 * @struct RobotPosition
 * @brief Represents the robot's position and heading
 */
struct RobotPosition {
    double x;           // X position in inches/units
    double y;           // Y position in inches/units
    double heading;     // Heading in radians
    
    RobotPosition() : x(0), y(0), heading(0) {}
    RobotPosition(double _x, double _y, double _heading) : x(_x), y(_y), heading(_heading) {}
};

/**
 * @class Odometry
 * @brief Handles position tracking using tracking wheels and IMU
 */
class Odometry {
public:
    /**
     * @brief Initialize the odometry system
     */
    static void initialize();
    
    /**
     * @brief Start the odometry update task
     */
    static void startTask();
    
    /**
     * @brief Stop the odometry update task
     */
    static void stopTask();
    
    /**
     * @brief Set the current position
     */
    static void setPosition(double x, double y, double heading);
    
    /**
     * @brief Set the current position
     */
    static void setPosition(const RobotPosition& pos);
    
    /**
     * @brief Get the current position (thread-safe)
     */
    static RobotPosition getPosition();
    
    /**
     * @brief Get the current X position
     */
    static double getX();
    
    /**
     * @brief Get the current Y position
     */
    static double getY();
    
    /**
     * @brief Get the current heading in radians
     */
    static double getHeading();
    
    /**
     * @brief Get the current heading in degrees
     */
    static double getHeadingDegrees();
    
    /**
     * @brief Reset the odometry to origin
     */
    static void reset();
    
    /**
     * @brief Check if the odometry task is running
     */
    static bool isRunning();
    
    /**
     * @brief Get distance traveled since last reset
     */
    static double getTotalDistance();
    
    /**
     * @brief Check if robot is stationary
     * @param threshold Movement threshold
     * @return Number of consecutive cycles with no movement
     */
    static int getStationaryCount();
    
    /**
     * @brief Enable/disable debug output
     */
    static void setDebug(bool enable);

private:
    /**
     * @brief The odometry update function (runs in separate task)
     */
    static void updateTask(void* param);
    
    /**
     * @brief Single update cycle
     */
    static void update();
    
    // Current position (atomic for thread safety)
    static std::atomic<double> s_x;
    static std::atomic<double> s_y;
    static std::atomic<double> s_heading;
    
    // Previous encoder values
    static double s_lastVerticalEncoder;
    static double s_lastHorizontalEncoder;
    static double s_lastHeading;
    
    // Task handle
    static pros::Task* s_task;
    static std::atomic<bool> s_running;
    
    // Statistics
    static double s_totalDistance;
    static int s_stationaryCount;
    
    // Debug flag
    static bool s_debug;
    
    // Mutex for position updates
    static pros::Mutex s_mutex;
};

//==========================================================================
// HARDWARE OBJECTS (extern declarations)
//==========================================================================

namespace Hardware {
    // IMU
    extern pros::IMU imu;
    
    // Tracking wheel encoders
    extern pros::Rotation verticalEncoder;
    extern pros::Rotation horizontalEncoder;
    
    // High scoring rotation sensor
    extern pros::Rotation highScoringRotation;
    
    // Color sensor
    extern pros::Optical colorSensor;
    
    // Drivetrain motors
    extern pros::Motor leftMotorA;
    extern pros::Motor leftMotorB;
    extern pros::Motor leftMotorC;
    extern pros::Motor rightMotorA;
    extern pros::Motor rightMotorB;
    extern pros::Motor rightMotorC;
    
    // Motor groups
    extern pros::MotorGroup leftDrive;
    extern pros::MotorGroup rightDrive;
    
    // Mechanism motors
    extern pros::Motor highScoringMotor;
    extern pros::Motor intakeLower;
    extern pros::Motor intakeUpper;
    
    // ADI (3-wire) devices - using new API
    extern pros::adi::DigitalOut mogoClamp;
    extern pros::adi::DigitalOut ejection;
    extern pros::adi::DigitalOut doinker;
    extern pros::adi::DigitalOut intakePneumatic;
    
    // Controller
    extern pros::Controller master;
    
    /**
     * @brief Initialize all hardware devices
     */
    void initialize();
}

#endif // ODOMETRY_HPP
