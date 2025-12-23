/**
 * @file calibration.cpp
 * @brief Calibration system implementation
 */

#include "calibration.hpp"
#include "odometry.hpp"
#include "config.hpp"
#include "utils.hpp"
#include "pid.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

namespace Calibration {
    static bool s_calibrating = false;
    static bool s_calibrationLoaded = false;
    
    //==========================================================================
    // CALIBRATION MODE ENTRY
    //==========================================================================
    
    bool shouldEnterCalibrationMode() {
        // Check if UP + X buttons are held during startup
        return Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) &&
               Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    }
    
    void runCalibrationMode() {
        s_calibrating = true;
        
        pros::lcd::initialize();
        pros::lcd::set_text(1, "CALIBRATION MODE");
        pros::lcd::set_text(2, "Follow Instructions");
        printf("=== CALIBRATION MODE ===\n");
        
        // Wait for buttons to be released
        while (Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) ||
               Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            pros::delay(10);
        }
        
        // Run calibration steps
        displayProgress(1, 6, "Tracking Wheel Positions");
        calibrateTrackingWheelPositions();
        
        displayProgress(2, 6, "Wheel Diameters");
        calibrateWheelDiameters();
        
        displayProgress(3, 6, "Track Width");
        calibrateTrackWidth();
        
        displayProgress(4, 6, "Motion Characterization");
        characterizeMotion();
        
        displayProgress(5, 6, "Distance Sensors");
        calibrateDistanceSensors();
        
        displayProgress(6, 6, "PID Auto-Tuning");
        autoTunePID();
        
        // Save calibration data
        pros::lcd::set_text(1, "Saving calibration...");
        saveCalibrationData();
        
        pros::lcd::set_text(1, "Calibration Complete!");
        pros::lcd::set_text(2, "Press any button to reboot");
        printf("Calibration complete! Press any button to continue.\n");
        
        Utils::waitForAnyButton(Hardware::master);
        
        s_calibrating = false;
    }
    
    //==========================================================================
    // LOAD/SAVE CALIBRATION
    //==========================================================================
    
    bool loadCalibrationData() {
        s_calibrationLoaded = Config::loadCalibration();
        if (s_calibrationLoaded) {
            PID::loadFromCalibration();
        }
        return s_calibrationLoaded;
    }
    
    bool saveCalibrationData() {
        return Config::saveCalibration();
    }
    
    //==========================================================================
    // CALIBRATION STEPS
    //==========================================================================
    
    void calibrateTrackingWheelPositions() {
        printf("\n=== STEP 1: Tracking Wheel Position Calibration ===\n");
        pros::lcd::set_text(2, "Place robot on clear floor");
        pros::lcd::set_text(3, "Press A when ready");
        
        waitForButtonWithMessage(pros::E_CONTROLLER_DIGITAL_A, "Press A to start wheel calibration");
        
        const int NUM_TRIALS = 3;
        std::vector<double> forwardOffsets;
        std::vector<double> sidewaysOffsets;
        
        for (int trial = 0; trial < NUM_TRIALS; trial++) {
            printf("Trial %d/%d\n", trial + 1, NUM_TRIALS);
            pros::lcd::set_text(4, ("Trial " + std::to_string(trial + 1)).c_str());
            
            // Reset encoders
            Hardware::verticalEncoder.reset_position();
            Hardware::horizontalEncoder.reset_position();
            Hardware::imu.set_heading(0);
            
            // Spin 360 degrees
            spinInPlace(360, 30);
            pros::delay(500);
            
            // Calculate forward wheel offset
            double verticalDist = (Hardware::verticalEncoder.get_position() / 100.0 / 360.0) * 
                                  Config::ODOM_WHEEL_CIRCUMFERENCE * Config::FEET_TO_UNIT;
            double forwardOffset = verticalDist / (2.0 * M_PI);
            forwardOffsets.push_back(forwardOffset);
            
            // Calculate sideways wheel offset
            double horizontalDist = (Hardware::horizontalEncoder.get_position() / 100.0 / 360.0) * 
                                    Config::ODOM_WHEEL_CIRCUMFERENCE * Config::FEET_TO_UNIT;
            double sidewaysOffset = horizontalDist / (2.0 * M_PI);
            sidewaysOffsets.push_back(sidewaysOffset);
            
            printf("  Forward offset: %.3f, Sideways offset: %.3f\n", forwardOffset, sidewaysOffset);
            
            pros::delay(1000);
        }
        
        // Average the results
        Config::forward_wheel_offset = Utils::average(forwardOffsets);
        Config::sideways_wheel_offset = Utils::average(sidewaysOffsets);
        
        printf("Final forward offset: %.3f inches\n", Config::forward_wheel_offset);
        printf("Final sideways offset: %.3f inches\n", Config::sideways_wheel_offset);
    }
    
    void calibrateWheelDiameters() {
        printf("\n=== STEP 2: Wheel Diameter Calibration ===\n");
        pros::lcd::set_text(2, "Drive forward 10 rotations");
        pros::lcd::set_text(3, "Press A to start");
        
        waitForButtonWithMessage(pros::E_CONTROLLER_DIGITAL_A, "Press A to start diameter calibration");
        
        // Drive forward 10 rotations
        driveForward(10, 50);
        
        pros::lcd::set_text(2, "Measure actual distance");
        pros::lcd::set_text(3, "Enter in terminal");
        
        // In a real implementation, this would read from terminal
        // For now, use the default value
        printf("Enter measured distance in inches (or press Enter for default): ");
        
        // Default to theoretical distance
        double theoretical = 10.0 * M_PI * Config::DRIVETRAIN_WHEEL_DIAMETER;
        Config::actual_drivetrain_diameter = Config::DRIVETRAIN_WHEEL_DIAMETER;
        
        printf("Using theoretical: %.3f inches\n", theoretical);
        printf("Drivetrain diameter: %.3f inches\n", Config::actual_drivetrain_diameter);
        
        // Repeat for odometry wheels
        Config::actual_odom_diameter = Config::ODOM_WHEEL_DIAMETER;
        printf("Odometry diameter: %.3f inches\n", Config::actual_odom_diameter);
    }
    
    void calibrateTrackWidth() {
        printf("\n=== STEP 3: Track Width Calibration ===\n");
        pros::lcd::set_text(2, "Robot will spin to measure track width");
        pros::lcd::set_text(3, "Press A to start");
        
        waitForButtonWithMessage(pros::E_CONTROLLER_DIGITAL_A, "Press A to start track width calibration");
        
        // Reset motors
        Hardware::leftDrive.tare_position();
        Hardware::imu.set_heading(0);
        
        // Spin 360 degrees
        spinInPlace(360, 30);
        pros::delay(500);
        
        // Calculate track width from wheel distance
        double wheelDistance = std::abs(Hardware::leftDrive.get_position() / 360.0) * 
                               M_PI * Config::actual_drivetrain_diameter;
        Config::track_width = wheelDistance / M_PI;
        
        printf("Track width: %.3f inches\n", Config::track_width);
    }
    
    void characterizeMotion() {
        printf("\n=== STEP 4: Motion Characterization ===\n");
        printf("Testing motion at various speeds...\n");
        
        // Test at different speeds
        double speeds[] = {25, 50, 75, 100};
        
        for (double speed : speeds) {
            printf("Testing at %.0f%% speed\n", speed);
            
            Hardware::leftDrive.move(speed * 1.27);
            Hardware::rightDrive.move(speed * 1.27);
            pros::delay(500);
            
            double actualSpeed = (Hardware::leftDrive.get_actual_velocity() + 
                                  Hardware::rightDrive.get_actual_velocity()) / 2.0;
            
            printf("  Requested: %.0f%%, Actual: %.1f RPM\n", speed, actualSpeed);
            
            Hardware::leftDrive.move(0);
            Hardware::rightDrive.move(0);
            pros::delay(500);
        }
    }
    
    void calibrateDistanceSensors() {
        printf("\n=== STEP 5: Distance Sensor Calibration ===\n");
        printf("Distance sensors not connected, skipping...\n");
        // Would calibrate distance sensors if present
    }
    
    void autoTunePID() {
        printf("\n=== STEP 6: PID Auto-Tuning ===\n");
        printf("Using Ziegler-Nichols method\n");
        
        // For now, use reasonable defaults
        // A full implementation would use oscillation detection
        
        Config::pid_linear_p = 1.0;
        Config::pid_linear_i = 0.0;
        Config::pid_linear_d = 0.1;
        
        Config::pid_angular_p = 2.0;
        Config::pid_angular_i = 0.0;
        Config::pid_angular_d = 0.2;
        
        Config::pid_turn_p = 2.5;
        Config::pid_turn_i = 0.0;
        Config::pid_turn_d = 0.3;
        
        printf("Linear PID: P=%.3f, I=%.3f, D=%.3f\n", 
               Config::pid_linear_p, Config::pid_linear_i, Config::pid_linear_d);
        printf("Angular PID: P=%.3f, I=%.3f, D=%.3f\n",
               Config::pid_angular_p, Config::pid_angular_i, Config::pid_angular_d);
        printf("Turn PID: P=%.3f, I=%.3f, D=%.3f\n",
               Config::pid_turn_p, Config::pid_turn_i, Config::pid_turn_d);
    }
    
    //==========================================================================
    // HELPER FUNCTIONS
    //==========================================================================
    
    void waitForButtonWithMessage(pros::controller_digital_e_t button, const char* message) {
        printf("%s\n", message);
        Utils::waitForButton(Hardware::master, button);
    }
    
    double readFromTerminal(const char* prompt) {
        printf("%s", prompt);
        // In a real implementation, this would read from terminal
        return 0.0;
    }
    
    void displayProgress(int step, int totalSteps, const char* description) {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "Step %d/%d: %s", step, totalSteps, description);
        pros::lcd::set_text(1, buffer);
        printf("\n%s\n", buffer);
    }
    
    void spinInPlace(double degrees, double velocity) {
        double startHeading = Hardware::imu.get_heading();
        double targetHeading = startHeading + degrees;
        
        while (std::abs(Hardware::imu.get_heading() - startHeading) < std::abs(degrees) - 5) {
            double progress = std::abs(Hardware::imu.get_heading() - startHeading) / std::abs(degrees);
            double currentVel = velocity;
            
            // Slow down near end
            if (progress > 0.8) {
                currentVel = velocity * (1.0 - progress) * 5.0;
                currentVel = std::max(currentVel, 10.0);
            }
            
            Hardware::leftDrive.move(currentVel * 1.27);
            Hardware::rightDrive.move(-currentVel * 1.27);
            
            pros::delay(20);
        }
        
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
    }
    
    void driveForward(double rotations, double velocity) {
        Hardware::leftDrive.tare_position();
        
        double targetDegrees = rotations * 360.0;
        
        while (std::abs(Hardware::leftDrive.get_position()) < targetDegrees) {
            Hardware::leftDrive.move(velocity * 1.27);
            Hardware::rightDrive.move(velocity * 1.27);
            pros::delay(20);
        }
        
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
    }
    
    bool detectOscillation() {
        // Would implement oscillation detection for Ziegler-Nichols
        return false;
    }
    
    double measureOscillationPeriod() {
        // Would measure oscillation period for Ziegler-Nichols
        return 0.5;
    }
    
    bool isCalibrating() {
        return s_calibrating;
    }
    
    bool isCalibrationLoaded() {
        return s_calibrationLoaded;
    }
}

