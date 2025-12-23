/**
 * @file test_with_mock.cpp
 * @brief Test robot code with mocked PROS API
 * 
 * This allows testing your robot code logic without hardware.
 * Compile with: clang++ -std=c++17 -I../include -I. test_with_mock.cpp -o test_with_mock
 */

#define TEST_MODE
#include "mock_pros.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

// Mock the PROS API header
#define _PROS_API_H_

// Test basic motor functionality
void test_motor() {
    std::cout << "Testing Motor... ";
    
    pros::Motor motor(1, pros::v5::MotorGears::blue);
    motor.move(100);
    assert(motor.get_voltage() == 100);
    
    motor.move_velocity(300);
    assert(motor.get_actual_velocity() == 300);
    
    motor.set_brake_mode(pros::v5::MotorBrake::brake);
    assert(motor.get_brake_mode() == pros::v5::MotorBrake::brake);
    
    std::cout << "PASSED\n";
}

// Test motor group
void test_motor_group() {
    std::cout << "Testing MotorGroup... ";
    
    pros::MotorGroup mg({1, 2, 3}, pros::v5::MotorGears::blue);
    mg.move(100);
    mg.set_brake_mode_all(pros::v5::MotorBrake::brake);
    
    std::cout << "PASSED\n";
}

// Test controller
void test_controller() {
    std::cout << "Testing Controller... ";
    
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    // Test button press
    master.press_button(pros::E_CONTROLLER_DIGITAL_A);
    assert(master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == true);
    
    // Test new press
    assert(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == true);
    assert(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == false);
    
    // Test analog
    master.set_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y, 100);
    assert(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) == 100);
    
    std::cout << "PASSED\n";
}

// Test IMU
void test_imu() {
    std::cout << "Testing IMU... ";
    
    pros::IMU imu(9);
    imu.reset();
    assert(imu.is_calibrating() == true);
    
    imu.set_heading(90);
    assert(imu.is_calibrating() == false);
    assert(std::abs(imu.get_heading() - 90) < 0.1);
    
    imu.simulate_rotation(45);
    assert(std::abs(imu.get_heading() - 135) < 0.1);
    
    std::cout << "PASSED\n";
}

// Test rotation sensor
void test_rotation() {
    std::cout << "Testing Rotation sensor... ";
    
    pros::Rotation rot(7);
    assert(rot.get_position() == 0);
    
    rot.simulate_rotation(90);
    assert(std::abs(rot.get_position() - 9000) < 1); // 90 degrees = 9000 centidegrees
    
    rot.reset_position();
    assert(rot.get_position() == 0);
    
    std::cout << "PASSED\n";
}

// Test ADI DigitalOut
void test_adi() {
    std::cout << "Testing ADI DigitalOut... ";
    
    pros::adi::DigitalOut out('A');
    assert(out.get_value() == false);
    
    out.set_value(true);
    assert(out.get_value() == true);
    
    std::cout << "PASSED\n";
}

// Simulate a simple robot movement
void test_robot_simulation() {
    std::cout << "Testing Robot Simulation... ";
    
    // Create mock hardware
    pros::Motor leftMotor(-2, pros::v5::MotorGears::blue);
    pros::Motor rightMotor(5, pros::v5::MotorGears::blue);
    pros::IMU imu(9);
    pros::Rotation verticalEncoder(7);
    pros::Rotation horizontalEncoder(6);
    
    // Initialize
    imu.set_heading(0);
    verticalEncoder.reset_position();
    horizontalEncoder.reset_position();
    
    // Simulate driving forward
    leftMotor.move(100);
    rightMotor.move(100);
    
    // Simulate time passing (0.1 seconds)
    leftMotor.update(0.1);
    rightMotor.update(0.1);
    
    // Simulate encoder movement (wheel rotates 180 degrees)
    verticalEncoder.simulate_rotation(180);
    
    // Check that position changed
    assert(verticalEncoder.get_position() > 0);
    
    std::cout << "PASSED\n";
}

int main() {
    std::cout << "\n========================================\n";
    std::cout << "PROS Mock API Tests\n";
    std::cout << "========================================\n\n";
    
    test_motor();
    test_motor_group();
    test_controller();
    test_imu();
    test_rotation();
    test_adi();
    test_robot_simulation();
    
    std::cout << "\n========================================\n";
    std::cout << "All mock tests PASSED!\n";
    std::cout << "========================================\n\n";
    
    std::cout << "Note: This tests the mock API, not your actual robot code.\n";
    std::cout << "To test your robot code, you would need to:\n";
    std::cout << "1. Include your source files\n";
    std::cout << "2. Use #define TEST_MODE to conditionally compile\n";
    std::cout << "3. Link against mock_pros.hpp instead of real PROS\n\n";
    
    return 0;
}

