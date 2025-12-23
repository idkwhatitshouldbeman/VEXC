/**
 * @file pid.hpp
 * @brief PID Controller implementation
 */

#ifndef PID_HPP
#define PID_HPP

#include "api.h"

/**
 * @class PIDController
 * @brief A configurable PID controller with anti-windup
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID Controller
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param maxIntegral Maximum integral accumulation (anti-windup)
     */
    PIDController(double kP = 0.0, double kI = 0.0, double kD = 0.0, double maxIntegral = 50.0);
    
    /**
     * @brief Calculate PID output
     * @param target Target/setpoint value
     * @param current Current/measured value
     * @return PID output
     */
    double calculate(double target, double current);
    
    /**
     * @brief Calculate PID output with error input
     * @param error Error value (target - current)
     * @return PID output
     */
    double calculateFromError(double error);
    
    /**
     * @brief Reset the controller state
     */
    void reset();
    
    /**
     * @brief Set PID gains
     */
    void setGains(double kP, double kI, double kD);
    
    /**
     * @brief Set maximum integral value (anti-windup)
     */
    void setMaxIntegral(double max);
    
    /**
     * @brief Set output limits
     */
    void setOutputLimits(double min, double max);
    
    /**
     * @brief Get current proportional term
     */
    double getP() const { return m_pTerm; }
    
    /**
     * @brief Get current integral term
     */
    double getI() const { return m_iTerm; }
    
    /**
     * @brief Get current derivative term
     */
    double getD() const { return m_dTerm; }
    
    /**
     * @brief Get last error
     */
    double getLastError() const { return m_lastError; }
    
    /**
     * @brief Check if target is reached within tolerance
     */
    bool isSettled(double tolerance) const;
    
    // Getters for gains
    double getKP() const { return m_kP; }
    double getKI() const { return m_kI; }
    double getKD() const { return m_kD; }

private:
    double m_kP;
    double m_kI;
    double m_kD;
    
    double m_integral;
    double m_lastError;
    double m_maxIntegral;
    
    double m_pTerm;
    double m_iTerm;
    double m_dTerm;
    
    double m_minOutput;
    double m_maxOutput;
    bool m_hasOutputLimits;
    
    uint32_t m_lastTime;
};

//==========================================================================
// GLOBAL PID CONTROLLERS
//==========================================================================

namespace PID {
    // Forward declarations of global PID controllers
    extern PIDController linear;    // For linear movement
    extern PIDController angular;   // For heading correction
    extern PIDController turn;      // For point turns
    
    /**
     * @brief Initialize all PID controllers with config values
     */
    void initialize();
    
    /**
     * @brief Load PID values from calibration
     */
    void loadFromCalibration();
}

#endif // PID_HPP

