/**
 * @file main.cpp
 * @brief Main entry point for VEX V5 robot program
 * 
 * This is the main entry point for the robot program using PROS.
 * It handles initialization, competition callbacks, and mode management.
 */

#include "main.h"
#include <cstdio>

/**
 * @brief Runs initialization code
 * 
 * This function runs when the robot powers on. It should be used to
 * initialize hardware and prepare the robot for operation.
 */
void initialize() {
    printf("\n\n========================================\n");
    printf("VEX V5 Robot Initializing...\n");
    printf("========================================\n\n");
    
    // Initialize LCD
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Initializing...");
    
    // Check for calibration mode
    if (Calibration::shouldEnterCalibrationMode()) {
        printf("Entering calibration mode...\n");
        Calibration::runCalibrationMode();
        return;
    }
    
    // Initialize configuration with defaults
    Config::initDefaults();
    
    // Load calibration data from SD card
    if (Calibration::loadCalibrationData()) {
        printf("Calibration data loaded successfully\n");
    } else {
        printf("Using default calibration values\n");
    }
    
    // Initialize hardware
    printf("Initializing hardware...\n");
    Hardware::initialize();
    
    // Initialize subsystems
    printf("Initializing subsystems...\n");
    PID::initialize();
    PID::loadFromCalibration();
    Odometry::initialize();
    Intake::initialize();
    HighScoring::initialize();
    Pneumatics::initialize();
    ColorDetection::initialize();
    DriverControl::initialize();
    Autonomous::initialize();
    Telemetry::initialize();
    
    // Start background tasks
    printf("Starting background tasks...\n");
    Odometry::startTask();
    Telemetry::startTask();
    
    // Ready message
    pros::lcd::clear();
    pros::lcd::set_text(0, "Robot Ready!");
    pros::lcd::set_text(1, "Calibration: OK");
    pros::lcd::set_text(2, "Waiting for match...");
    
    printf("\n========================================\n");
    printf("Robot Ready!\n");
    printf("========================================\n\n");
}

/**
 * @brief Runs while the robot is disabled
 * 
 * This function runs repeatedly while the robot is disabled.
 * It's called approximately every 10ms.
 */
void disabled() {
    // Could update displays here
}

/**
 * @brief Runs at the start of competition connect
 * 
 * This function runs once when the competition switch is connected.
 * It's useful for autonomous selection.
 */
void competition_initialize() {
    printf("Competition connected\n");
    
    pros::lcd::set_text(0, "Competition Mode");
    pros::lcd::set_text(1, "Select autonomous:");
    pros::lcd::set_text(2, "UP=Red LEFT=Blue");
    
    // Simple autonomous selection using controller buttons
    // In a real implementation, you might use the LCD or a more sophisticated selector
    
    while (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
        // Check for autonomous selection
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            // Red side
            if (Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                Autonomous::setSlot(Autonomous::Slot::RED_LEFT);
                pros::lcd::set_text(3, "Selected: RED LEFT");
            } else {
                Autonomous::setSlot(Autonomous::Slot::RED_RIGHT);
                pros::lcd::set_text(3, "Selected: RED RIGHT");
            }
        }
        
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            // Blue side
            if (Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                Autonomous::setSlot(Autonomous::Slot::BLUE_LEFT);
                pros::lcd::set_text(3, "Selected: BLUE LEFT");
            } else {
                Autonomous::setSlot(Autonomous::Slot::BLUE_RIGHT);
                pros::lcd::set_text(3, "Selected: BLUE RIGHT");
            }
        }
        
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            Autonomous::setSlot(Autonomous::Slot::SKILLS);
            pros::lcd::set_text(3, "Selected: SKILLS");
        }
        
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            Autonomous::setSlot(Autonomous::Slot::TEST);
            pros::lcd::set_text(3, "Selected: TEST");
        }
        
        pros::delay(20);
    }
}

/**
 * @brief Runs the user autonomous code
 * 
 * This function runs during the autonomous period of a VEX competition.
 * It's called once when autonomous starts.
 */
void autonomous() {
    printf("\n========================================\n");
    printf("Autonomous Starting\n");
    printf("========================================\n\n");
    
    pros::lcd::set_text(0, "AUTONOMOUS");
    
    // Reset position for autonomous
    Odometry::reset();
    
    // Run the selected autonomous routine
    Autonomous::run();
    
    printf("Autonomous Complete\n");
}

/**
 * @brief Runs the operator control code
 * 
 * This function runs during the driver control period of a VEX competition.
 * It's called once when driver control starts.
 */
void opcontrol() {
    printf("\n========================================\n");
    printf("Driver Control Starting\n");
    printf("========================================\n\n");
    
    pros::lcd::set_text(0, "DRIVER CONTROL");
    
    // Main driver control loop
    while (true) {
        // Update all driver controls
        DriverControl::update();
        
        // Check for manual autonomous trigger (for testing)
        // Hold A+B to trigger autonomous (only when not in competition)
        if (!pros::competition::is_connected()) {
            if (Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
                Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                printf("Manual autonomous trigger\n");
                Autonomous::run();
            }
        }
        
        // Update displays
        Telemetry::updateBrainScreen();
        
        pros::delay(Config::DRIVER_LOOP_DELAY);
    }
}

