/**
 * @file odometry.cpp
 * @brief Odometry system implementation
 */

#include "odometry.hpp"
#include "config.hpp"
#include "utils.hpp"
#include <cstdio>
#include <cmath>

//==========================================================================
// HARDWARE OBJECTS
//==========================================================================

namespace Hardware {
    // IMU
    pros::IMU imu(Config::IMU_PORT);
    
    // Tracking wheel encoders
    pros::Rotation verticalEncoder(Config::VERTICAL_ENCODER_PORT);
    pros::Rotation horizontalEncoder(Config::HORIZONTAL_ENCODER_PORT);
    
    // High scoring rotation sensor
    pros::Rotation highScoringRotation(Config::HIGH_SCORING_ROTATION_PORT);
    
    // Color sensor
    pros::Optical colorSensor(Config::COLOR_SENSOR_PORT);
    
    // Drivetrain motors (negative port = reversed)
    // Left motors: ports 2, 3, 4 - REVERSED (use negative ports)
    // Right motors: ports 5, 12, 17 - NOT reversed
    pros::Motor leftMotorA(-Config::LEFT_MOTOR_A_PORT, pros::v5::MotorGears::blue);
    pros::Motor leftMotorB(-Config::LEFT_MOTOR_B_PORT, pros::v5::MotorGears::blue);
    pros::Motor leftMotorC(-Config::LEFT_MOTOR_C_PORT, pros::v5::MotorGears::blue);
    pros::Motor rightMotorA(Config::RIGHT_MOTOR_A_PORT, pros::v5::MotorGears::blue);
    pros::Motor rightMotorB(Config::RIGHT_MOTOR_B_PORT, pros::v5::MotorGears::blue);
    pros::Motor rightMotorC(Config::RIGHT_MOTOR_C_PORT, pros::v5::MotorGears::blue);
    
    // Motor groups - use port numbers directly (negative for reversed)
    pros::MotorGroup leftDrive({-Config::LEFT_MOTOR_A_PORT, -Config::LEFT_MOTOR_B_PORT, -Config::LEFT_MOTOR_C_PORT}, 
                                pros::v5::MotorGears::blue);
    pros::MotorGroup rightDrive({Config::RIGHT_MOTOR_A_PORT, Config::RIGHT_MOTOR_B_PORT, Config::RIGHT_MOTOR_C_PORT}, 
                                 pros::v5::MotorGears::blue);
    
    // Mechanism motors - Green cartridge (200 RPM)
    pros::Motor highScoringMotor(Config::HIGH_SCORING_MOTOR_PORT, pros::v5::MotorGears::green);
    pros::Motor intakeLower(Config::INTAKE_LOWER_PORT, pros::v5::MotorGears::green);
    pros::Motor intakeUpper(Config::INTAKE_UPPER_PORT, pros::v5::MotorGears::green);
    
    // ADI (3-wire) devices - using new API (pros::adi::DigitalOut)
    pros::adi::DigitalOut mogoClamp(Config::MOGO_CLAMP_PORT);
    pros::adi::DigitalOut ejection(Config::EJECTION_PORT);
    pros::adi::DigitalOut doinker(Config::DOINKER_PORT);
    pros::adi::DigitalOut intakePneumatic(Config::INTAKE_PNEUMATIC_PORT);
    
    // Controller
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    void initialize() {
        // Set motor brake modes using new enum
        leftDrive.set_brake_mode_all(pros::MotorBrake::brake);
        rightDrive.set_brake_mode_all(pros::MotorBrake::brake);
        highScoringMotor.set_brake_mode(pros::MotorBrake::brake);
        intakeLower.set_brake_mode(pros::MotorBrake::coast);
        intakeUpper.set_brake_mode(pros::MotorBrake::coast);
        
        // Calibrate IMU
        printf("Calibrating IMU...\n");
        imu.reset();
        while (imu.is_calibrating()) {
            pros::delay(20);
        }
        printf("IMU calibrated!\n");
        
        // Reset encoders
        verticalEncoder.reset_position();
        horizontalEncoder.reset_position();
        highScoringRotation.reset_position();
        
        // Initialize color sensor
        colorSensor.set_led_pwm(100);
        
        // Initialize pneumatics to default state
        mogoClamp.set_value(false);
        ejection.set_value(false);
        doinker.set_value(false);
    }
}

//==========================================================================
// ODOMETRY CLASS
//==========================================================================

// Static member initialization
std::atomic<double> Odometry::s_x(0);
std::atomic<double> Odometry::s_y(0);
std::atomic<double> Odometry::s_heading(0);

double Odometry::s_lastVerticalEncoder = 0;
double Odometry::s_lastHorizontalEncoder = 0;
double Odometry::s_lastHeading = 0;

pros::Task* Odometry::s_task = nullptr;
std::atomic<bool> Odometry::s_running(false);

double Odometry::s_totalDistance = 0;
int Odometry::s_stationaryCount = 0;

bool Odometry::s_debug = false;

pros::Mutex Odometry::s_mutex;

void Odometry::initialize() {
    // Reset position to origin
    reset();
    
    // Reset encoders
    Hardware::verticalEncoder.reset_position();
    Hardware::horizontalEncoder.reset_position();
    
    s_lastVerticalEncoder = 0;
    s_lastHorizontalEncoder = 0;
    s_lastHeading = 0;
}

void Odometry::startTask() {
    if (s_task == nullptr) {
        s_running = true;
        s_task = new pros::Task(updateTask, nullptr, "Odometry");
        printf("Odometry task started\n");
    }
}

void Odometry::stopTask() {
    s_running = false;
    if (s_task != nullptr) {
        s_task->remove();
        delete s_task;
        s_task = nullptr;
        printf("Odometry task stopped\n");
    }
}

void Odometry::setPosition(double x, double y, double heading) {
    s_mutex.take(TIMEOUT_MAX);
    s_x = x;
    s_y = y;
    s_heading = heading;
    
    // Reset encoders
    Hardware::verticalEncoder.reset_position();
    Hardware::horizontalEncoder.reset_position();
    Hardware::imu.set_heading(Utils::radToDeg(heading));
    
    s_lastVerticalEncoder = 0;
    s_lastHorizontalEncoder = 0;
    s_lastHeading = heading;
    
    s_mutex.give();
}

void Odometry::setPosition(const RobotPosition& pos) {
    setPosition(pos.x, pos.y, pos.heading);
}

RobotPosition Odometry::getPosition() {
    return RobotPosition(s_x, s_y, s_heading);
}

double Odometry::getX() {
    return s_x;
}

double Odometry::getY() {
    return s_y;
}

double Odometry::getHeading() {
    return s_heading;
}

double Odometry::getHeadingDegrees() {
    return Utils::radToDeg(s_heading);
}

void Odometry::reset() {
    setPosition(0, 0, 0);
    s_totalDistance = 0;
    s_stationaryCount = 0;
}

bool Odometry::isRunning() {
    return s_running;
}

double Odometry::getTotalDistance() {
    return s_totalDistance;
}

int Odometry::getStationaryCount() {
    return s_stationaryCount;
}

void Odometry::setDebug(bool enable) {
    s_debug = enable;
}

void Odometry::updateTask(void* param) {
    while (s_running) {
        update();
        pros::delay(Config::ODOMETRY_LOOP_DELAY);
    }
}

void Odometry::update() {
    // Get current encoder readings (convert to inches)
    double verticalEncoderDeg = Hardware::verticalEncoder.get_position() / 100.0;  // centidegrees to degrees
    double horizontalEncoderDeg = Hardware::horizontalEncoder.get_position() / 100.0;
    
    double verticalEncoder = (verticalEncoderDeg / 360.0) * Config::ODOM_WHEEL_CIRCUMFERENCE * 
                             Config::ODOM_GEAR_RATIO * Config::FEET_TO_UNIT;
    double horizontalEncoder = (horizontalEncoderDeg / 360.0) * Config::ODOM_WHEEL_CIRCUMFERENCE * 
                               Config::ODOM_GEAR_RATIO * Config::FEET_TO_UNIT;
    
    // Get current heading from IMU (convert to radians)
    // Using 2*PI - radians because of coordinate system (from reference code)
    double currentHeading = 2.0 * M_PI - Utils::degToRad(Hardware::imu.get_heading());
    
    // Calculate deltas
    double deltaVertical = verticalEncoder - s_lastVerticalEncoder;
    double deltaHorizontal = horizontalEncoder - s_lastHorizontalEncoder;
    double deltaHeading = Utils::normalizeAngle(currentHeading - s_lastHeading);
    
    // Check if robot is stationary
    if (std::abs(deltaVertical) < 0.001 && std::abs(deltaHorizontal) < 0.001) {
        s_stationaryCount++;
    } else {
        s_stationaryCount = 0;
    }
    
    // Account for tracking wheel offsets during rotation
    double forwardOffset = Config::forward_wheel_offset * deltaHeading;
    double sidewaysOffset = Config::sideways_wheel_offset * deltaHeading;
    
    // Correct for the offset
    double correctedVertical = deltaVertical - forwardOffset;
    double correctedHorizontal = deltaHorizontal - sidewaysOffset;
    
    // Calculate local position change using average heading
    double avgHeading = currentHeading - (deltaHeading / 2.0);
    
    // Convert local movement to global coordinates
    // Vertical movement (forward/backward)
    double deltaX_vertical = correctedVertical * std::cos(avgHeading);
    double deltaY_vertical = correctedVertical * std::sin(avgHeading);
    
    // Horizontal movement (sideways)
    double deltaX_horizontal = correctedHorizontal * std::cos(avgHeading + M_PI / 2.0);
    double deltaY_horizontal = correctedHorizontal * std::sin(avgHeading + M_PI / 2.0);
    
    // Apply both movements with mutex protection
    s_mutex.take(TIMEOUT_MAX);
    s_x = s_x + deltaX_vertical + deltaX_horizontal;
    s_y = s_y + deltaY_vertical + deltaY_horizontal;
    s_heading = currentHeading;
    s_mutex.give();
    
    // Update total distance
    s_totalDistance += std::abs(correctedVertical) + std::abs(correctedHorizontal);
    
    // Update last values
    s_lastVerticalEncoder = verticalEncoder;
    s_lastHorizontalEncoder = horizontalEncoder;
    s_lastHeading = currentHeading;
    
    // Debug output
    if (s_debug) {
        printf("x: %.2f y: %.2f angle: %.2f\n", (double)s_x, (double)s_y, (double)s_heading);
    }
}
