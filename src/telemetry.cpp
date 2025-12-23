/**
 * @file telemetry.cpp
 * @brief Telemetry and logging implementation
 */

#include "telemetry.hpp"
#include "odometry.hpp"
#include "mechanisms.hpp"
#include "driver_control.hpp"
#include "autonomous.hpp"
#include "config.hpp"
#include "utils.hpp"
#include <cstdio>
#include <cstdarg>
#include <cstring>

namespace Telemetry {
    static pros::Task* s_task = nullptr;
    static bool s_running = false;
    static bool s_debugEnabled = true;
    static bool s_logging = false;
    static uint32_t s_startTime = 0;
    static int s_logCounter = 0;
    
    //==========================================================================
    // TASK MANAGEMENT
    //==========================================================================
    
    static void telemetryTask(void* param) {
        while (s_running) {
            printStatus();
            updateBrainScreen();
            updateControllerScreen();
            
            if (s_logging) {
                logState();
            }
            
            pros::delay(Config::TELEMETRY_LOOP_DELAY);
        }
    }
    
    void initialize() {
        s_startTime = pros::millis();
        s_debugEnabled = true;
        s_logging = false;
        s_logCounter = 0;
        
        pros::lcd::initialize();
    }
    
    void startTask() {
        if (s_task == nullptr) {
            s_running = true;
            s_task = new pros::Task(telemetryTask, nullptr, "Telemetry");
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
    
    //==========================================================================
    // TERMINAL OUTPUT
    //==========================================================================
    
    void printStatus() {
        static int printCycle = 0;
        printCycle++;
        
        // Only print every 10 cycles (1 Hz)
        if (printCycle % 10 != 0) return;
        
        RobotPosition pos = Odometry::getPosition();
        
        printf("=== ROBOT STATUS ===\n");
        printf("Position: (%.2f, %.2f) heading: %.1f deg\n", 
               pos.x, pos.y, Utils::radToDeg(pos.heading));
        printf("Battery: %.1fV (%d%%)\n", 
               getBatteryVoltage(), getBatteryCapacity());
        printf("Mode: %s\n", 
               DriverControl::isRunning() ? "Driver" : 
               Autonomous::isRunning() ? "Auto" : "Disabled");
        printf("\n");
    }
    
    void print(const char* format, ...) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
    
    void debug(const char* format, ...) {
        if (!s_debugEnabled) return;
        
        va_list args;
        va_start(args, format);
        printf("[DEBUG] ");
        vprintf(format, args);
        va_end(args);
    }
    
    void setDebugEnabled(bool enabled) {
        s_debugEnabled = enabled;
    }
    
    bool isDebugEnabled() {
        return s_debugEnabled;
    }
    
    //==========================================================================
    // BRAIN SCREEN DISPLAY
    //==========================================================================
    
    void updateBrainScreen() {
        RobotPosition pos = Odometry::getPosition();
        
        char line1[32], line2[32], line3[32], line4[32], line5[32];
        
        snprintf(line1, sizeof(line1), "X: %.1f  Y: %.1f", pos.x, pos.y);
        snprintf(line2, sizeof(line2), "Heading: %.1f deg", Utils::radToDeg(pos.heading));
        snprintf(line3, sizeof(line3), "Battery: %d%%", getBatteryCapacity());
        snprintf(line4, sizeof(line4), "Intake: %s", 
                 Intake::getState() == IntakeState::RUNNING ? "ON" :
                 Intake::getState() == IntakeState::STALLED ? "STALL" : "OFF");
        snprintf(line5, sizeof(line5), "Mogo: %s", 
                 Pneumatics::isMogoClamped() ? "CLAMPED" : "OPEN");
        
        pros::lcd::set_text(1, line1);
        pros::lcd::set_text(2, line2);
        pros::lcd::set_text(3, line3);
        pros::lcd::set_text(4, line4);
        pros::lcd::set_text(5, line5);
    }
    
    void clearBrainScreen() {
        pros::lcd::clear();
    }
    
    void displayMessage(int line, const char* format, ...) {
        char buffer[32];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        pros::lcd::set_text(line, buffer);
    }
    
    void displayError(const char* message) {
        pros::lcd::clear();
        pros::lcd::set_text(1, "ERROR:");
        pros::lcd::set_text(2, message);
        pros::lcd::set_text(3, "Check terminal");
        
        printf("ERROR: %s\n", message);
    }
    
    void displayPosition() {
        RobotPosition pos = Odometry::getPosition();
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.0f", 
                 pos.x, pos.y, Utils::radToDeg(pos.heading));
        pros::lcd::set_text(1, buffer);
    }
    
    void displayMode() {
        const char* mode = DriverControl::isRunning() ? "DRIVER" :
                          Autonomous::isRunning() ? "AUTO" : "DISABLED";
        pros::lcd::set_text(0, mode);
    }
    
    //==========================================================================
    // CONTROLLER DISPLAY
    //==========================================================================
    
    void updateControllerScreen() {
        static int displayCycle = 0;
        displayCycle++;
        
        // Update every 5 cycles (500ms)
        if (displayCycle % 5 != 0) return;
        
        RobotPosition pos = Odometry::getPosition();
        
        char line0[20], line1[20], line2[20];
        snprintf(line0, sizeof(line0), "Batt: %d%%", getBatteryCapacity());
        snprintf(line1, sizeof(line1), "%.0f,%.0f", pos.x, pos.y);
        snprintf(line2, sizeof(line2), "H:%.0f", Utils::radToDeg(pos.heading));
        
        Hardware::master.set_text(0, 0, line0);
        Hardware::master.set_text(1, 0, line1);
        Hardware::master.set_text(2, 0, line2);
    }
    
    void setControllerLine(int line, const char* text) {
        Hardware::master.set_text(line, 0, text);
    }
    
    void clearControllerScreen() {
        Hardware::master.clear();
    }
    
    //==========================================================================
    // SD CARD LOGGING
    //==========================================================================
    
    void startLogging() {
        if (s_logging) return;
        
        s_logging = true;
        s_logCounter = 0;
        
        // Write header
        std::string header = "time_ms,x,y,heading,battery\n";
        Utils::writeFile(Config::LOG_FILE, header);
        
        printf("Logging started to %s\n", Config::LOG_FILE);
    }
    
    void stopLogging() {
        s_logging = false;
        printf("Logging stopped\n");
    }
    
    bool isLogging() {
        return s_logging;
    }
    
    void logState() {
        s_logCounter++;
        
        // Only log every 10 cycles (1 Hz)
        if (s_logCounter % 10 != 0) return;
        
        RobotPosition pos = Odometry::getPosition();
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%u,%.3f,%.3f,%.3f,%.2f\n",
                 pros::millis(),
                 pos.x,
                 pos.y,
                 pos.heading,
                 getBatteryVoltage());
        
        Utils::appendFile(Config::LOG_FILE, buffer);
    }
    
    void log(const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        Utils::appendFile(Config::LOG_FILE, buffer);
    }
    
    void logPosition() {
        RobotPosition pos = Odometry::getPosition();
        log("%.3f,%.3f,%.3f\n", pos.x, pos.y, pos.heading);
    }
    
    std::string getLogFilename() {
        return Config::LOG_FILE;
    }
    
    //==========================================================================
    // STATISTICS
    //==========================================================================
    
    double getBatteryVoltage() {
        return pros::battery::get_voltage() / 1000.0;
    }
    
    double getBatteryCurrent() {
        return pros::battery::get_current() / 1000.0;
    }
    
    int getBatteryCapacity() {
        return static_cast<int>(pros::battery::get_capacity());
    }
    
    double getMotorTemperature(int port) {
        pros::Motor motor(port);
        return motor.get_temperature();
    }
    
    bool checkSystemHealth() {
        // Check battery
        if (getBatteryCapacity() < 20) {
            displayError("Low battery!");
            return false;
        }
        
        // Check motor temperatures
        double maxTemp = 0;
        int ports[] = {Config::LEFT_MOTOR_A_PORT, Config::LEFT_MOTOR_B_PORT, 
                       Config::LEFT_MOTOR_C_PORT, Config::RIGHT_MOTOR_A_PORT,
                       Config::RIGHT_MOTOR_B_PORT, Config::RIGHT_MOTOR_C_PORT};
        
        for (int port : ports) {
            double temp = getMotorTemperature(port);
            if (temp > maxTemp) maxTemp = temp;
        }
        
        if (maxTemp > 55) {
            displayError("Motor overheating!");
            return false;
        }
        
        return true;
    }
    
    uint32_t getUptime() {
        return pros::millis() - s_startTime;
    }
}

