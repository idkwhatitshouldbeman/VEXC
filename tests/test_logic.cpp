/**
 * @file test_logic.cpp
 * @brief Basic logic tests for robot subsystems
 * 
 * This file can be compiled with a standard C++ compiler to test
 * the mathematical logic without VEX hardware.
 * 
 * Compile with: g++ -std=c++17 -I../include -DTEST_MODE test_logic.cpp -o test_logic
 */

#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

// Mock definitions for testing without PROS
#define M_PI 3.14159265358979323846

namespace Utils {
    double degToRad(double deg) { return deg * M_PI / 180.0; }
    double radToDeg(double rad) { return rad * 180.0 / M_PI; }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    double distance(double x1, double y1, double x2, double y2) {
        return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    }
    
    double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    
    double cubicScale(double input, double maxOutput) {
        double normalized = input / 127.0;
        double cubed = normalized * normalized * normalized;
        return cubed * maxOutput;
    }
    
    double applyDeadband(double value, double deadband) {
        if (std::abs(value) < deadband) return 0.0;
        return value;
    }
}

// PID Controller for testing
class PIDController {
public:
    double kP, kI, kD;
    double integral = 0;
    double prevError = 0;
    double maxIntegral = 1000;
    double maxOutput = 100;
    
    PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}
    
    double calculate(double error, double dt = 0.02) {
        integral += error * dt;
        integral = Utils::clamp(integral, -maxIntegral, maxIntegral);
        
        double derivative = (error - prevError) / dt;
        prevError = error;
        
        double output = (kP * error) + (kI * integral) + (kD * derivative);
        return Utils::clamp(output, -maxOutput, maxOutput);
    }
    
    void reset() {
        integral = 0;
        prevError = 0;
    }
};

//==========================================================================
// TESTS
//==========================================================================

void testAngleConversion() {
    std::cout << "Testing angle conversion... ";
    
    assert(std::abs(Utils::degToRad(180) - M_PI) < 0.001);
    assert(std::abs(Utils::degToRad(90) - M_PI/2) < 0.001);
    assert(std::abs(Utils::radToDeg(M_PI) - 180) < 0.001);
    
    std::cout << "PASSED\n";
}

void testNormalizeAngle() {
    std::cout << "Testing angle normalization... ";
    
    assert(std::abs(Utils::normalizeAngle(0)) < 0.001);
    assert(std::abs(Utils::normalizeAngle(M_PI)) - M_PI < 0.001);
    assert(std::abs(Utils::normalizeAngle(3 * M_PI) - M_PI) < 0.001);
    assert(std::abs(Utils::normalizeAngle(-3 * M_PI) + M_PI) < 0.001);
    
    std::cout << "PASSED\n";
}

void testDistance() {
    std::cout << "Testing distance calculation... ";
    
    assert(std::abs(Utils::distance(0, 0, 3, 4) - 5.0) < 0.001);
    assert(std::abs(Utils::distance(0, 0, 0, 0)) < 0.001);
    assert(std::abs(Utils::distance(1, 1, 4, 5) - 5.0) < 0.001);
    
    std::cout << "PASSED\n";
}

void testClamp() {
    std::cout << "Testing clamp... ";
    
    assert(Utils::clamp(50, 0, 100) == 50);
    assert(Utils::clamp(-10, 0, 100) == 0);
    assert(Utils::clamp(150, 0, 100) == 100);
    
    std::cout << "PASSED\n";
}

void testCubicScale() {
    std::cout << "Testing cubic scaling... ";
    
    assert(std::abs(Utils::cubicScale(0, 100)) < 0.001);
    assert(std::abs(Utils::cubicScale(127, 100) - 100) < 0.001);
    assert(std::abs(Utils::cubicScale(-127, 100) + 100) < 0.001);
    // Mid-range should be reduced (cubic curve)
    assert(std::abs(Utils::cubicScale(63.5, 100)) < 15); // ~12.5
    
    std::cout << "PASSED\n";
}

void testDeadband() {
    std::cout << "Testing deadband... ";
    
    assert(Utils::applyDeadband(3, 5) == 0);
    assert(Utils::applyDeadband(10, 5) == 10);
    assert(Utils::applyDeadband(-3, 5) == 0);
    
    std::cout << "PASSED\n";
}

void testPIDController() {
    std::cout << "Testing PID controller... ";
    
    PIDController pid(1.0, 0.1, 0.05);
    
    // With error, should produce output
    double output = pid.calculate(10);
    assert(output > 0);
    
    // Zero error should produce near-zero P component
    pid.reset();
    output = pid.calculate(0);
    assert(std::abs(output) < 1);
    
    // Integral should accumulate
    pid.reset();
    pid.calculate(10);
    pid.calculate(10);
    pid.calculate(10);
    // After 3 iterations, integral should be non-zero
    assert(pid.integral > 0);
    
    std::cout << "PASSED\n";
}

void testOdometryMath() {
    std::cout << "Testing odometry math... ";
    
    // Test forward movement calculation
    double heading = 0; // Facing +X
    double deltaForward = 10;
    
    double deltaX = deltaForward * std::cos(heading);
    double deltaY = deltaForward * std::sin(heading);
    
    assert(std::abs(deltaX - 10) < 0.001);
    assert(std::abs(deltaY) < 0.001);
    
    // Test 45 degree movement
    heading = M_PI / 4;
    deltaX = deltaForward * std::cos(heading);
    deltaY = deltaForward * std::sin(heading);
    
    assert(std::abs(deltaX - deltaY) < 0.001); // Should be equal at 45 deg
    
    std::cout << "PASSED\n";
}

void testPurePursuitLookahead() {
    std::cout << "Testing pure pursuit lookahead geometry... ";
    
    // Robot at origin, target at (10, 0)
    double robotX = 0, robotY = 0;
    double targetX = 10, targetY = 0;
    double lookahead = 5;
    
    double dist = Utils::distance(robotX, robotY, targetX, targetY);
    assert(dist == 10);
    
    // With lookahead of 5, lookahead point should be at (5, 0)
    double t = lookahead / dist;
    double lookaheadX = robotX + t * (targetX - robotX);
    double lookaheadY = robotY + t * (targetY - robotY);
    
    assert(std::abs(lookaheadX - 5) < 0.001);
    assert(std::abs(lookaheadY) < 0.001);
    
    std::cout << "PASSED\n";
}

void testCurvatureCalculation() {
    std::cout << "Testing curvature calculation... ";
    
    // Robot at origin facing +X, target at (5, 5)
    double robotX = 0, robotY = 0;
    double heading = 0;
    double targetX = 5, targetY = 5;
    
    // Calculate angle to target
    double angleToTarget = std::atan2(targetY - robotY, targetX - robotX);
    double angleError = Utils::normalizeAngle(angleToTarget - heading);
    
    // Should be 45 degrees (PI/4)
    assert(std::abs(angleError - M_PI/4) < 0.001);
    
    // Calculate curvature (2 * sin(angle) / distance)
    double dist = Utils::distance(robotX, robotY, targetX, targetY);
    double curvature = 2 * std::sin(angleError) / dist;
    
    // Curvature should be positive (turn left)
    assert(curvature > 0);
    
    std::cout << "PASSED\n";
}

int main() {
    std::cout << "\n========================================\n";
    std::cout << "VEX Robot Logic Tests\n";
    std::cout << "========================================\n\n";
    
    testAngleConversion();
    testNormalizeAngle();
    testDistance();
    testClamp();
    testCubicScale();
    testDeadband();
    testPIDController();
    testOdometryMath();
    testPurePursuitLookahead();
    testCurvatureCalculation();
    
    std::cout << "\n========================================\n";
    std::cout << "All tests PASSED!\n";
    std::cout << "========================================\n\n";
    
    return 0;
}

