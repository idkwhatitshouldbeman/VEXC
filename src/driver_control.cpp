/**
 * @file driver_control.cpp
 * @brief Driver control implementation
 */

#include "driver_control.hpp"
#include "odometry.hpp"
#include "mechanisms.hpp"
#include "config.hpp"
#include "utils.hpp"
#include <cstdio>
#include <cmath>

namespace DriverControl {
    static MechanismMode s_currentMode = MechanismMode::NONE;
    static bool s_slowDrive = false;
    static pros::Task* s_task = nullptr;
    static bool s_running = false;
    
    //==========================================================================
    // TASK MANAGEMENT
    //==========================================================================
    
    static void driverTask(void* param) {
        while (s_running) {
            update();
            pros::delay(Config::DRIVER_LOOP_DELAY);
        }
    }
    
    void initialize() {
        s_currentMode = MechanismMode::NONE;
        s_slowDrive = false;
    }
    
    void startTask() {
        if (s_task == nullptr) {
            s_running = true;
            s_task = new pros::Task(driverTask, nullptr, "DriverControl");
        }
    }
    
    void stopTask() {
        s_running = false;
        if (s_task != nullptr) {
            s_task->remove();
            delete s_task;
            s_task = nullptr;
        }
    }
    
    bool isRunning() {
        return s_running;
    }
    
    void update() {
        setDriveVelocities();
        handleHighScoringButtons();
        HighScoring::update();
        handleIntakeButtons();
        ColorDetection::update();
        Intake::update();
        handlePneumaticButtons();
    }
    
    //==========================================================================
    // DRIVE CONTROL
    //==========================================================================
    
    void setDriveVelocities() {
        // Check for slow drive toggle
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            s_slowDrive = !s_slowDrive;
        }
        
        // Get joystick values
        int leftY = Hardware::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = Hardware::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        // Apply cubic scaling
        double leftScaled = Utils::cubicScale(leftY, s_slowDrive ? 90.0 : 100.0);
        double rightScaled = Utils::cubicScale(rightY, s_slowDrive ? 90.0 : 100.0);
        
        // Apply deadband
        leftScaled = Utils::applyDeadband(leftScaled, 5.0);
        rightScaled = Utils::applyDeadband(rightScaled, 5.0);
        
        // Set motor velocities (convert percent to RPM for 600 RPM motors)
        if (std::abs(leftScaled) < 5.0) {
            Hardware::leftDrive.move(0);
        } else {
            Hardware::leftDrive.move(leftScaled * 1.27);  // Convert percent to voltage (-127 to 127)
        }
        
        if (std::abs(rightScaled) < 5.0) {
            Hardware::rightDrive.move(0);
        } else {
            Hardware::rightDrive.move(rightScaled * 1.27);
        }
    }
    
    void toggleSlowDrive() {
        s_slowDrive = !s_slowDrive;
    }
    
    bool isSlowDriveEnabled() {
        return s_slowDrive;
    }
    
    void stopDrive() {
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
    }
    
    //==========================================================================
    // MECHANISM CONTROL
    //==========================================================================
    
    MechanismMode getCurrentMode() {
        return s_currentMode;
    }
    
    void setCurrentMode(MechanismMode mode) {
        s_currentMode = mode;
    }
    
    void handleModeSelection() {
        // X Button: TOP SCORING
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            s_currentMode = MechanismMode::TOP_MODE;
            Intake::collect();
            Hardware::master.rumble(".");
            displayMessage("TOP MODE");
        }
        
        // LEFT + A Button: MID SCORING (A alone is used for slow drive toggle)
        if (Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && 
            Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            s_currentMode = MechanismMode::MID_MODE;
            Hardware::master.rumble(".");
            displayMessage("MID MODE");
        }
        
        // B Button: BOTTOM SCORING
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            s_currentMode = MechanismMode::BOTTOM_MODE;
            Hardware::master.rumble(".");
            displayMessage("BOTTOM MODE");
        }
        
        // Y Button: INTAKE OVERRIDE
        if (Hardware::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            s_currentMode = MechanismMode::INTAKE_OVERRIDE;
            Intake::collect();
        } else if (s_currentMode == MechanismMode::INTAKE_OVERRIDE) {
            Intake::stop();
            s_currentMode = MechanismMode::NONE;
        }
    }
    
    void handleActionButtons() {
        // R1 and R2: Execute scoring based on mode
        if (s_currentMode == MechanismMode::TOP_MODE) {
            if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                Scoring::execute(Scoring::Position::TOP_BACK);
                s_currentMode = MechanismMode::NONE;
            } else if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                Scoring::execute(Scoring::Position::TOP_FRONT);
                s_currentMode = MechanismMode::NONE;
            }
        } else if (s_currentMode == MechanismMode::MID_MODE) {
            if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                Scoring::execute(Scoring::Position::MID_BACK);
                s_currentMode = MechanismMode::NONE;
            } else if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                Scoring::execute(Scoring::Position::MID_FRONT);
                s_currentMode = MechanismMode::NONE;
            }
        } else if (s_currentMode == MechanismMode::BOTTOM_MODE) {
            if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) ||
                Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                Scoring::execute(Scoring::Position::BOTTOM);
                s_currentMode = MechanismMode::NONE;
            }
        }
    }
    
    void handleIntakeButtons() {
        // R1: Intake forward (toggle)
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (Intake::getState() == IntakeState::STOPPED) {
                Intake::collect();
            } else {
                Intake::stop();
            }
        }
        
        // R2: Intake reverse (toggle)
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (Intake::getState() == IntakeState::STOPPED) {
                Intake::eject();
            } else {
                Intake::stop();
            }
        }
    }
    
    void handleHighScoringButtons() {
        // Left: Score position
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            HighScoring::setTarget(HighScoringTarget::SCORE);
            HighScoring::setRunning(false);
        }
        
        // Up: Wait position
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            HighScoring::setTarget(HighScoringTarget::WAIT);
            HighScoring::setRunning(false);
        }
        
        // Right: Capture position
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            HighScoring::setTarget(HighScoringTarget::CAPTURE);
            HighScoring::setRunning(true);
        }
        
        // Down: Down position
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            HighScoring::setTarget(HighScoringTarget::DOWN);
            HighScoring::setRunning(false);
        }
    }
    
    void handlePneumaticButtons() {
        // L1: Mogo clamp release
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            Pneumatics::clampMogo(false);
            printf("Mogo released\n");
        }
        
        // L2: Mogo clamp engage
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            Pneumatics::clampMogo(true);
            printf("Mogo clamped\n");
        }
        
        // X: Toggle intake pneumatic
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            Pneumatics::toggleIntakePneumatic();
        }
        
        // Y: Doinker on
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            Pneumatics::setDoinker(true);
        }
        
        // B: Doinker off
        if (Hardware::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            Pneumatics::setDoinker(false);
        }
    }
    
    //==========================================================================
    // DISPLAY
    //==========================================================================
    
    void updateControllerDisplay() {
        static int displayCycle = 0;
        displayCycle++;
        
        // Update display every 10 cycles (200ms at 20ms loop)
        if (displayCycle % 10 != 0) return;
        
        // Line 0: Battery
        char line0[20];
        snprintf(line0, sizeof(line0), "Batt: %d%%", (int)pros::battery::get_capacity());
        Hardware::master.set_text(0, 0, line0);
        
        // Line 1: Position
        RobotPosition pos = Odometry::getPosition();
        char line1[20];
        snprintf(line1, sizeof(line1), "%.0f,%.0f", pos.x, pos.y);
        Hardware::master.set_text(1, 0, line1);
        
        // Line 2: Mode
        const char* modeStr = "NONE";
        switch (s_currentMode) {
            case MechanismMode::TOP_MODE: modeStr = "TOP"; break;
            case MechanismMode::MID_MODE: modeStr = "MID"; break;
            case MechanismMode::BOTTOM_MODE: modeStr = "BOT"; break;
            case MechanismMode::INTAKE_OVERRIDE: modeStr = "INTAKE"; break;
            default: break;
        }
        Hardware::master.set_text(2, 0, modeStr);
    }
    
    void displayMessage(const char* message) {
        Hardware::master.clear();
        Hardware::master.set_text(1, 0, message);
    }
}

