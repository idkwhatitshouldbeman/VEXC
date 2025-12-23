/**
 * @file pid.cpp
 * @brief PID Controller implementation
 */

#include "pid.hpp"
#include "config.hpp"
#include "utils.hpp"

//==========================================================================
// PID CONTROLLER CLASS
//==========================================================================

PIDController::PIDController(double kP, double kI, double kD, double maxIntegral)
    : m_kP(kP), m_kI(kI), m_kD(kD),
      m_integral(0), m_lastError(0), m_maxIntegral(maxIntegral),
      m_pTerm(0), m_iTerm(0), m_dTerm(0),
      m_minOutput(-100), m_maxOutput(100), m_hasOutputLimits(false),
      m_lastTime(0) {
}

double PIDController::calculate(double target, double current) {
    return calculateFromError(target - current);
}

double PIDController::calculateFromError(double error) {
    // Calculate time delta
    uint32_t currentTime = pros::millis();
    double dt = (m_lastTime > 0) ? (currentTime - m_lastTime) / 1000.0 : 0.02;
    m_lastTime = currentTime;
    
    // Proportional term
    m_pTerm = m_kP * error;
    
    // Integral term with anti-windup
    m_integral += error * dt;
    if (m_integral > m_maxIntegral) m_integral = m_maxIntegral;
    if (m_integral < -m_maxIntegral) m_integral = -m_maxIntegral;
    m_iTerm = m_kI * m_integral;
    
    // Derivative term
    double derivative = (dt > 0) ? (error - m_lastError) / dt : 0.0;
    m_dTerm = m_kD * derivative;
    m_lastError = error;
    
    // Calculate output
    double output = m_pTerm + m_iTerm + m_dTerm;
    
    // Apply output limits if set
    if (m_hasOutputLimits) {
        output = Utils::clamp(output, m_minOutput, m_maxOutput);
    }
    
    return output;
}

void PIDController::reset() {
    m_integral = 0;
    m_lastError = 0;
    m_pTerm = 0;
    m_iTerm = 0;
    m_dTerm = 0;
    m_lastTime = 0;
}

void PIDController::setGains(double kP, double kI, double kD) {
    m_kP = kP;
    m_kI = kI;
    m_kD = kD;
}

void PIDController::setMaxIntegral(double max) {
    m_maxIntegral = max;
}

void PIDController::setOutputLimits(double min, double max) {
    m_minOutput = min;
    m_maxOutput = max;
    m_hasOutputLimits = true;
}

bool PIDController::isSettled(double tolerance) const {
    return std::abs(m_lastError) < tolerance;
}

//==========================================================================
// GLOBAL PID CONTROLLERS
//==========================================================================

namespace PID {
    // Define global PID controllers
    PIDController linear(Config::LINEAR_KP, Config::LINEAR_KI, Config::LINEAR_KD);
    PIDController angular(Config::ANGULAR_KP, Config::ANGULAR_KI, Config::ANGULAR_KD);
    PIDController turn(Config::TURN_KP, Config::TURN_KI, Config::TURN_KD);
    
    void initialize() {
        linear.setOutputLimits(-100, 100);
        angular.setOutputLimits(-100, 100);
        turn.setOutputLimits(-100, 100);
    }
    
    void loadFromCalibration() {
        linear.setGains(Config::pid_linear_p, Config::pid_linear_i, Config::pid_linear_d);
        angular.setGains(Config::pid_angular_p, Config::pid_angular_i, Config::pid_angular_d);
        turn.setGains(Config::pid_turn_p, Config::pid_turn_i, Config::pid_turn_d);
    }
}

