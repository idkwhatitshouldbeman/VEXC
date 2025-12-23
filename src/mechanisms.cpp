/**
 * @file mechanisms.cpp
 * @brief Mechanism control implementation
 */

#include "mechanisms.hpp"
#include "odometry.hpp"
#include "config.hpp"
#include "utils.hpp"
#include <cstdio>
#include <cmath>

//==========================================================================
// INTAKE SYSTEM
//==========================================================================

namespace Intake {
    static IntakeState s_state = IntakeState::STOPPED;
    static RingType s_ejectColor = RingType::NONE;
    static bool s_currentReverse = false;
    static int s_consecutiveStallCount = 0;
    static int s_retryCount = 0;
    static int s_ejectCounter = 0;
    static bool s_highScoreStall = false;
    
    void initialize() {
        s_state = IntakeState::STOPPED;
        s_consecutiveStallCount = 0;
        s_retryCount = 0;
        s_ejectCounter = 0;
        
        Hardware::intakeLower.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Hardware::intakeUpper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    
    void setState(IntakeState state, bool reverse) {
        s_state = state;
        s_currentReverse = reverse;
        
        if (state == IntakeState::RUNNING || state == IntakeState::FIXING_STALL) {
            int lowerVel = Config::INTAKE_LOWER_VELOCITY;
            int upperVel = Config::INTAKE_UPPER_VELOCITY;
            
            if (reverse) {
                Hardware::intakeLower.move_velocity(-lowerVel * 2);  // Convert to RPM
                Hardware::intakeUpper.move_velocity(upperVel * 2);
            } else {
                Hardware::intakeLower.move_velocity(lowerVel * 2);
                Hardware::intakeUpper.move_velocity(-upperVel * 2);
            }
            
            if (state == IntakeState::FIXING_STALL) {
                if (!s_highScoreStall) {
                    // Reverse upper motor to fix stall
                    if (reverse) {
                        Hardware::intakeUpper.move_velocity(-upperVel * 2);
                    } else {
                        Hardware::intakeUpper.move_velocity(upperVel * 2);
                    }
                } else {
                    s_highScoreStall = false;
                    Hardware::intakeUpper.move(0);
                }
            }
        } else {
            Hardware::intakeLower.move(0);
            Hardware::intakeUpper.move(0);
        }
    }
    
    IntakeState getState() {
        return s_state;
    }
    
    void update() {
        if (s_state == IntakeState::RUNNING || s_state == IntakeState::STALLED) {
            // Handle eject counter
            if (s_state == IntakeState::RUNNING && s_ejectCounter > 0) {
                s_ejectCounter--;
                if (s_ejectCounter == 0) {
                    s_state = IntakeState::STOPPED;
                    setState(IntakeState::STOPPED, s_currentReverse);
                    pros::delay(100);
                    return;
                }
            }
            
            // Check for stall
            double currentVelocity = Hardware::intakeUpper.get_actual_velocity();
            if (std::abs(currentVelocity) <= Config::STALL_THRESHOLD) {
                s_consecutiveStallCount++;
            } else {
                s_consecutiveStallCount = 0;
            }
            
            if (s_consecutiveStallCount >= Config::STALL_COUNT) {
                printf("Unstalling\n");
                
                if (HighScoring::isRunning()) {
                    printf("High scoring stall\n");
                    s_highScoreStall = true;
                    HighScoring::setTarget(HighScoringTarget::WAIT);
                    HighScoring::update();
                }
                
                setState(IntakeState::FIXING_STALL, s_currentReverse);
                s_consecutiveStallCount = 0;
                s_retryCount = Config::RETRY_LIMIT;
            }
        } else {
            s_consecutiveStallCount = 0;
        }
        
        if (s_state == IntakeState::FIXING_STALL) {
            if (s_retryCount == 0) {
                if (s_highScoreStall) {
                    printf("Stopping due to high stall\n");
                    s_state = IntakeState::STOPPED;
                    setState(IntakeState::STOPPED, false);
                } else {
                    printf("Fixed stall\n");
                    s_state = IntakeState::RUNNING;
                    setState(IntakeState::RUNNING, s_currentReverse);
                }
            } else {
                s_retryCount--;
            }
        }
    }
    
    void collect() {
        s_consecutiveStallCount = 0;
        s_retryCount = 0;
        setState(IntakeState::RUNNING, false);
    }
    
    void eject() {
        s_consecutiveStallCount = 0;
        s_retryCount = 0;
        setState(IntakeState::RUNNING, true);
    }
    
    void stop() {
        setState(IntakeState::STOPPED, false);
    }
    
    bool isStalled() {
        return s_state == IntakeState::STALLED || s_state == IntakeState::FIXING_STALL;
    }
    
    void setEjectColor(RingType color) {
        s_ejectColor = color;
    }
    
    RingType getEjectColor() {
        return s_ejectColor;
    }
}

//==========================================================================
// HIGH SCORING MECHANISM
//==========================================================================

namespace HighScoring {
    static double s_targetAngle = Config::HIGH_SCORE_ANGLE_DOWN;
    static bool s_running = false;
    static int s_capturePositionCounter = 0;
    
    void initialize() {
        s_targetAngle = Config::HIGH_SCORE_ANGLE_DOWN;
        s_running = false;
        s_capturePositionCounter = 0;
        
        Hardware::highScoringMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        Hardware::highScoringRotation.reset_position();
    }
    
    void setTarget(HighScoringTarget target) {
        switch (target) {
            case HighScoringTarget::DOWN:
                s_targetAngle = Config::HIGH_SCORE_ANGLE_DOWN;
                s_running = false;
                break;
            case HighScoringTarget::CAPTURE:
                s_targetAngle = Config::HIGH_SCORE_ANGLE_CAPTURE;
                s_capturePositionCounter = Config::MAX_CAPTURE_POSITION_COUNT;
                s_running = true;
                break;
            case HighScoringTarget::WAIT:
                s_targetAngle = Config::HIGH_SCORE_ANGLE_WAIT;
                s_running = false;
                break;
            case HighScoringTarget::SCORE:
                // If already at or past score position (more negative), go further
                // HIGH_SCORE_ANGLE_SCORE is negative (e.g., -430), so >= means more negative
                if (s_targetAngle >= Config::HIGH_SCORE_ANGLE_SCORE) {
                    s_targetAngle -= 40.0;
                } else {
                    s_targetAngle = Config::HIGH_SCORE_ANGLE_SCORE;
                }
                s_running = false;
                break;
        }
    }
    
    void setTargetAngle(double angle) {
        s_targetAngle = angle;
    }
    
    double getTargetAngle() {
        return s_targetAngle;
    }
    
    double getCurrentAngle() {
        return Hardware::highScoringRotation.get_position() / 100.0;  // centidegrees to degrees
    }
    
    void update() {
        Intake::update();
        
        Hardware::highScoringMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        
        // Handle capture position synchronization
        if (s_targetAngle == Config::HIGH_SCORE_ANGLE_CAPTURE) {
            double motorPos = Hardware::highScoringMotor.get_position();
            double sensorPos = getCurrentAngle();
            
            if (std::abs(motorPos - sensorPos) > 2.0) {
                if (s_capturePositionCounter > 0) {
                    s_capturePositionCounter--;
                } else {
                    printf("Syncing motor position\n");
                    Hardware::highScoringMotor.set_zero_position(sensorPos);
                }
            }
        }
        
        // Move to target position
        Hardware::highScoringMotor.move_absolute(s_targetAngle, 100);
    }
    
    bool atTarget(double tolerance) {
        double current = Hardware::highScoringMotor.get_position();
        return std::abs(current - s_targetAngle) < tolerance;
    }
    
    bool isRunning() {
        return s_running;
    }
    
    void setRunning(bool running) {
        s_running = running;
    }
    
    void handleStall() {
        // High scoring stall handling is done in Intake::update()
    }
}

//==========================================================================
// PNEUMATICS
//==========================================================================

namespace Pneumatics {
    static bool s_mogoClamped = false;
    static bool s_ejecting = false;
    static bool s_doinkerExtended = false;
    static bool s_intakePneumaticExtended = false;
    
    void initialize() {
        s_mogoClamped = false;
        s_ejecting = false;
        s_doinkerExtended = false;
        s_intakePneumaticExtended = false;
        
        Hardware::mogoClamp.set_value(false);
        Hardware::ejection.set_value(false);
        Hardware::doinker.set_value(false);
        Hardware::intakePneumatic.set_value(false);
    }
    
    void clampMogo(bool clamp) {
        s_mogoClamped = clamp;
        Hardware::mogoClamp.set_value(clamp);
    }
    
    bool isMogoClamped() {
        return s_mogoClamped;
    }
    
    void toggleMogoClamp() {
        clampMogo(!s_mogoClamped);
    }
    
    void setEjection(bool eject) {
        s_ejecting = eject;
        Hardware::ejection.set_value(eject);
    }
    
    bool isEjecting() {
        return s_ejecting;
    }
    
    void setDoinker(bool extend) {
        s_doinkerExtended = extend;
        Hardware::doinker.set_value(extend);
    }
    
    bool isDoinkerExtended() {
        return s_doinkerExtended;
    }
    
    void toggleDoinker() {
        setDoinker(!s_doinkerExtended);
    }
    
    void setIntakePneumatic(bool extend) {
        s_intakePneumaticExtended = extend;
        Hardware::intakePneumatic.set_value(extend);
    }
    
    bool isIntakePneumaticExtended() {
        return s_intakePneumaticExtended;
    }
    
    void toggleIntakePneumatic() {
        setIntakePneumatic(!s_intakePneumaticExtended);
    }
}

//==========================================================================
// COLOR DETECTION
//==========================================================================

namespace ColorDetection {
    static RingType s_ejectColor = RingType::NONE;
    static RingType s_detectedRing = RingType::NONE;
    
    void initialize() {
        s_ejectColor = RingType::NONE;
        s_detectedRing = RingType::NONE;
        Hardware::colorSensor.set_led_pwm(100);
    }
    
    void update() {
        // Check brightness threshold
        if (Hardware::colorSensor.get_brightness() <= Config::BRIGHTNESS_THRESHOLD) {
            s_detectedRing = RingType::NONE;
            return;
        }
        
        // Get hue value
        double hue = Hardware::colorSensor.get_hue();
        
        // Detect ring color based on hue
        // Red is around 0-30 and 330-360
        // Blue is around 180-240
        if ((hue >= 0 && hue <= 30) || (hue >= 330 && hue <= 360)) {
            s_detectedRing = RingType::RED;
        } else if (hue >= 180 && hue <= 240) {
            s_detectedRing = RingType::BLUE;
        } else {
            s_detectedRing = RingType::NONE;
        }
        
        // Handle ejection
        if (s_ejectColor != RingType::NONE && s_detectedRing == s_ejectColor) {
            printf("Ejecting %s ring\n", s_ejectColor == RingType::RED ? "red" : "blue");
            Pneumatics::setEjection(true);
        } else if (s_detectedRing != RingType::NONE && s_detectedRing != s_ejectColor) {
            Pneumatics::setEjection(false);
        }
    }
    
    RingType getDetectedRing() {
        return s_detectedRing;
    }
    
    void setEjectColor(RingType color) {
        s_ejectColor = color;
    }
    
    RingType getEjectColor() {
        return s_ejectColor;
    }
}

//==========================================================================
// SCORING SEQUENCES
//==========================================================================

namespace Scoring {
    static bool s_scoring = false;
    
    void execute(Position position) {
        s_scoring = true;
        
        switch (position) {
            case Position::TOP_BACK:
            case Position::TOP_FRONT:
                // Use high scoring mechanism
                HighScoring::setTarget(HighScoringTarget::SCORE);
                while (!HighScoring::atTarget(5.0)) {
                    HighScoring::update();
                    pros::delay(20);
                }
                // Eject ring
                Hardware::intakeUpper.move_velocity(-200);
                pros::delay(300);
                Hardware::intakeUpper.move(0);
                // Return mechanism
                HighScoring::setTarget(HighScoringTarget::WAIT);
                break;
                
            case Position::MID_BACK:
            case Position::MID_FRONT:
                // Mid scoring uses similar sequence
                HighScoring::setTarget(HighScoringTarget::WAIT);
                while (!HighScoring::atTarget(5.0)) {
                    HighScoring::update();
                    pros::delay(20);
                }
                Hardware::intakeUpper.move_velocity(-200);
                pros::delay(300);
                Hardware::intakeUpper.move(0);
                break;
                
            case Position::BOTTOM:
                // Bottom scoring just runs intake in reverse
                Hardware::intakeLower.move_velocity(200);
                Hardware::intakeUpper.move_velocity(200);
                pros::delay(500);
                Hardware::intakeLower.move(0);
                Hardware::intakeUpper.move(0);
                break;
        }
        
        s_scoring = false;
    }
    
    bool isScoring() {
        return s_scoring;
    }
    
    void cancel() {
        s_scoring = false;
        Hardware::intakeLower.move(0);
        Hardware::intakeUpper.move(0);
    }
}

