/**
 * @file autonomous.cpp
 * @brief Autonomous routines and instruction system implementation
 */

#include "autonomous.hpp"
#include "odometry.hpp"
#include "mechanisms.hpp"
#include "config.hpp"
#include "utils.hpp"
#include "pid.hpp"
#include <cstdio>
#include <cmath>

namespace Autonomous {
    static Slot s_currentSlot = Slot::RED_LEFT;
    static bool s_running = false;
    
    //==========================================================================
    // INITIALIZATION
    //==========================================================================
    
    void initialize() {
        s_currentSlot = Slot::RED_LEFT;
        s_running = false;
    }
    
    void setSlot(Slot slot) {
        s_currentSlot = slot;
    }
    
    Slot getSlot() {
        return s_currentSlot;
    }
    
    //==========================================================================
    // RUN AUTONOMOUS
    //==========================================================================
    
    void run() {
        run(s_currentSlot);
    }
    
    void run(Slot slot) {
        s_running = true;
        
        switch (slot) {
            case Slot::RED_LEFT:
                Hardware::imu.set_heading(180);
                ColorDetection::setEjectColor(RingType::BLUE);
                redLeft();
                break;
            case Slot::RED_RIGHT:
                Hardware::imu.set_heading(180);
                ColorDetection::setEjectColor(RingType::BLUE);
                redRight();
                break;
            case Slot::BLUE_LEFT:
                ColorDetection::setEjectColor(RingType::NONE);
                blueLeft();
                break;
            case Slot::BLUE_RIGHT:
                ColorDetection::setEjectColor(RingType::RED);
                blueRight();
                break;
            case Slot::SKILLS:
                ColorDetection::setEjectColor(RingType::NONE);
                skills();
                break;
            case Slot::TEST:
                ColorDetection::setEjectColor(RingType::NONE);
                test();
                break;
        }
        
        // Cleanup
        ColorDetection::setEjectColor(RingType::NONE);
        Pneumatics::setEjection(false);
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
        
        s_running = false;
    }
    
    //==========================================================================
    // INSTRUCTION SYSTEM
    //==========================================================================
    
    std::vector<Instruction> loadInstructions(const std::string& filename) {
        std::vector<Instruction> instructions;
        
        std::string content = Utils::readFile(filename.c_str());
        if (content.empty()) {
            printf("Error: Could not load instructions from %s\n", filename.c_str());
            return instructions;
        }
        
        std::vector<std::string> lines = Utils::split(content, '\n');
        
        for (const std::string& rawLine : lines) {
            std::string line = Utils::trim(rawLine);
            
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') continue;
            
            // Remove inline comments
            size_t commentPos = line.find('#');
            if (commentPos != std::string::npos) {
                line = Utils::trim(line.substr(0, commentPos));
            }
            
            if (line.empty()) continue;
            
            // Parse instruction
            std::vector<std::string> tokens = Utils::split(line, ' ');
            if (tokens.empty()) continue;
            
            Instruction inst;
            
            // Determine instruction type
            const std::string& cmd = tokens[0];
            if (cmd == "INIT") inst.type = InstructionType::INIT;
            else if (cmd == "PATH") inst.type = InstructionType::PATH;
            else if (cmd == "WAIT_PATH") inst.type = InstructionType::WAIT_PATH;
            else if (cmd == "SCORE") inst.type = InstructionType::SCORE;
            else if (cmd == "INTAKE") inst.type = InstructionType::INTAKE;
            else if (cmd == "PTO") inst.type = InstructionType::PTO;
            else if (cmd == "WAIT") inst.type = InstructionType::WAIT;
            else if (cmd == "TURN_TO") inst.type = InstructionType::TURN_TO;
            else if (cmd == "DRIVE_TO") inst.type = InstructionType::DRIVE_TO;
            else if (cmd == "MOGO") inst.type = InstructionType::MOGO;
            else if (cmd == "DOINKER") inst.type = InstructionType::DOINKER;
            else if (cmd == "HIGH_SCORE") inst.type = InstructionType::HIGH_SCORE;
            else if (cmd == "SET_VELOCITY") inst.type = InstructionType::SET_VELOCITY;
            else if (cmd == "SET_EJECT") inst.type = InstructionType::SET_EJECT;
            else if (cmd == "PRINT") inst.type = InstructionType::PRINT;
            else if (cmd == "END") inst.type = InstructionType::END;
            else {
                printf("Unknown instruction: %s\n", cmd.c_str());
                continue;
            }
            
            // Store parameters
            for (size_t i = 1; i < tokens.size(); i++) {
                inst.params.push_back(tokens[i]);
            }
            
            instructions.push_back(inst);
        }
        
        printf("Loaded %d instructions\n", (int)instructions.size());
        return instructions;
    }
    
    void executeInstructions(const std::vector<Instruction>& instructions) {
        for (const Instruction& inst : instructions) {
            if (!s_running) break;
            executeInstruction(inst);
        }
    }
    
    void executeInstruction(const Instruction& inst) {
        switch (inst.type) {
            case InstructionType::INIT:
                if (inst.params.size() >= 3) {
                    double x = std::stod(inst.params[0]);
                    double y = std::stod(inst.params[1]);
                    double heading = Utils::degToRad(std::stod(inst.params[2]));
                    Odometry::setPosition(x, y, heading);
                    printf("INIT: %.2f, %.2f, %.2f\n", x, y, heading);
                }
                break;
                
            case InstructionType::PATH:
                if (inst.params.size() >= 1) {
                    std::string pathFile = "/usd/paths/" + inst.params[0];
                    Path path = PurePursuit::loadPathFromCSV(pathFile);
                    int direction = (inst.params.size() > 1 && inst.params[1] == "reverse") ? -1 : 1;
                    PurePursuit::followPath(path, direction);
                }
                break;
                
            case InstructionType::WAIT_PATH:
                while (PurePursuit::isFollowing()) {
                    pros::delay(50);
                }
                break;
                
            case InstructionType::SCORE:
                if (inst.params.size() >= 1) {
                    const std::string& pos = inst.params[0];
                    if (pos == "TOP_BACK") Scoring::execute(Scoring::Position::TOP_BACK);
                    else if (pos == "TOP_FRONT") Scoring::execute(Scoring::Position::TOP_FRONT);
                    else if (pos == "MID_BACK") Scoring::execute(Scoring::Position::MID_BACK);
                    else if (pos == "MID_FRONT") Scoring::execute(Scoring::Position::MID_FRONT);
                    else if (pos == "BOTTOM") Scoring::execute(Scoring::Position::BOTTOM);
                }
                break;
                
            case InstructionType::INTAKE:
                if (inst.params.size() >= 1) {
                    const std::string& state = inst.params[0];
                    if (state == "COLLECT") Intake::collect();
                    else if (state == "EJECT") Intake::eject();
                    else if (state == "STOP") Intake::stop();
                }
                break;
                
            case InstructionType::WAIT:
                if (inst.params.size() >= 1) {
                    int ms = std::stoi(inst.params[0]);
                    pros::delay(ms);
                }
                break;
                
            case InstructionType::TURN_TO:
                if (inst.params.size() >= 1) {
                    double degrees = std::stod(inst.params[0]);
                    turnToHeadingDeg(degrees);
                }
                break;
                
            case InstructionType::DRIVE_TO:
                if (inst.params.size() >= 2) {
                    double x = std::stod(inst.params[0]);
                    double y = std::stod(inst.params[1]);
                    driveToPoint(x, y);
                }
                break;
                
            case InstructionType::MOGO:
                if (inst.params.size() >= 1) {
                    bool on = (inst.params[0] == "on" || inst.params[0] == "true" || inst.params[0] == "1");
                    Pneumatics::clampMogo(on);
                }
                break;
                
            case InstructionType::DOINKER:
                if (inst.params.size() >= 1) {
                    bool on = (inst.params[0] == "on" || inst.params[0] == "true" || inst.params[0] == "1");
                    Pneumatics::setDoinker(on);
                }
                break;
                
            case InstructionType::HIGH_SCORE:
                if (inst.params.size() >= 1) {
                    const std::string& pos = inst.params[0];
                    if (pos == "DOWN") HighScoring::setTarget(HighScoringTarget::DOWN);
                    else if (pos == "CAPTURE") HighScoring::setTarget(HighScoringTarget::CAPTURE);
                    else if (pos == "WAIT") HighScoring::setTarget(HighScoringTarget::WAIT);
                    else if (pos == "SCORE") HighScoring::setTarget(HighScoringTarget::SCORE);
                }
                break;
                
            case InstructionType::SET_VELOCITY:
                if (inst.params.size() >= 1) {
                    double vel = std::stod(inst.params[0]);
                    PurePursuit::setForwardVelocity(vel);
                }
                break;
                
            case InstructionType::SET_EJECT:
                if (inst.params.size() >= 1) {
                    const std::string& color = inst.params[0];
                    if (color == "RED") ColorDetection::setEjectColor(RingType::RED);
                    else if (color == "BLUE") ColorDetection::setEjectColor(RingType::BLUE);
                    else ColorDetection::setEjectColor(RingType::NONE);
                }
                break;
                
            case InstructionType::PRINT:
                if (inst.params.size() >= 1) {
                    printf("AUTO: %s\n", inst.params[0].c_str());
                }
                break;
                
            case InstructionType::PTO:
                // PTO control if needed
                break;
                
            case InstructionType::END:
                s_running = false;
                break;
        }
    }
    
    //==========================================================================
    // BASIC MOVEMENT FUNCTIONS
    //==========================================================================
    
    void turnToHeading(double targetHeading, int timeout) {
        PID::turn.reset();
        
        uint32_t startTime = pros::millis();
        
        while (true) {
            RobotPosition pos = Odometry::getPosition();
            double error = Utils::normalizeAngle(targetHeading - pos.heading);
            
            // Exit condition
            if (std::abs(error) < 0.05) {  // ~3 degrees
                break;
            }
            
            // Timeout check
            if (timeout > 0 && (pros::millis() - startTime) > (uint32_t)timeout) {
                break;
            }
            
            double power = PID::turn.calculateFromError(error);
            power = Utils::clamp(power, -100.0, 100.0);
            
            Hardware::leftDrive.move(power * 1.27);
            Hardware::rightDrive.move(-power * 1.27);
            
            pros::delay(20);
        }
        
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
    }
    
    void turnToHeadingDeg(double targetDegrees, int timeout) {
        turnToHeading(Utils::degToRad(targetDegrees), timeout);
    }
    
    void driveToPoint(double targetX, double targetY, int timeout) {
        PID::linear.reset();
        PID::angular.reset();
        
        uint32_t startTime = pros::millis();
        
        while (true) {
            RobotPosition pos = Odometry::getPosition();
            
            double dx = targetX - pos.x;
            double dy = targetY - pos.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double angleToTarget = std::atan2(dy, dx);
            double angleError = Utils::normalizeAngle(angleToTarget - pos.heading);
            
            // Exit condition
            if (distance < 2.0) {  // 2 inches
                break;
            }
            
            // Timeout check
            if (timeout > 0 && (pros::millis() - startTime) > (uint32_t)timeout) {
                break;
            }
            
            double drivePower = PID::linear.calculate(distance, 0);
            double turnPower = PID::angular.calculateFromError(angleError);
            
            double leftPower = drivePower + turnPower;
            double rightPower = drivePower - turnPower;
            
            leftPower = Utils::clamp(leftPower, -100.0, 100.0);
            rightPower = Utils::clamp(rightPower, -100.0, 100.0);
            
            Hardware::leftDrive.move(leftPower * 1.27);
            Hardware::rightDrive.move(rightPower * 1.27);
            
            pros::delay(20);
        }
        
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
    }
    
    void driveDistance(double distance, double velocity) {
        double startDist = Odometry::getTotalDistance();
        double targetDist = std::abs(distance);
        int direction = (distance >= 0) ? 1 : -1;
        
        while (std::abs(Odometry::getTotalDistance() - startDist) < targetDist) {
            Hardware::leftDrive.move(velocity * direction * 1.27);
            Hardware::rightDrive.move(velocity * direction * 1.27);
            pros::delay(20);
        }
        
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
    }
    
    void waitForPath(int timeout) {
        uint32_t startTime = pros::millis();
        
        while (PurePursuit::isFollowing()) {
            if (timeout > 0 && (pros::millis() - startTime) > (uint32_t)timeout) {
                PurePursuit::stop();
                break;
            }
            pros::delay(50);
        }
    }
    
    //==========================================================================
    // STATE
    //==========================================================================
    
    bool isRunning() {
        return s_running;
    }
    
    void stop() {
        s_running = false;
        PurePursuit::stop();
        Hardware::leftDrive.move(0);
        Hardware::rightDrive.move(0);
    }
    
    //==========================================================================
    // PRE-DEFINED ROUTINES
    //==========================================================================
    
    void redLeft() {
        printf("Running Red Left autonomous\n");
        
        // Set high scoring to wait position
        HighScoring::setTarget(HighScoringTarget::WAIT);
        HighScoring::update();
        
        double lookahead = 50.0;
        double tolerance = 2.0;
        
        // This would normally load paths from SD card
        // For now, we'll use basic movements
        
        // Example: Drive backward to mogo, clamp, drive forward
        driveDistance(-24, 50);
        Pneumatics::clampMogo(true);
        pros::delay(100);
        
        Intake::collect();
        driveDistance(24, 50);
        
        // More autonomous actions would go here...
        
        HighScoring::setTarget(HighScoringTarget::DOWN);
        HighScoring::update();
        Intake::stop();
    }
    
    void redRight() {
        printf("Running Red Right autonomous\n");
        // Similar to redLeft but mirrored
        redLeft();  // Placeholder
    }
    
    void blueLeft() {
        printf("Running Blue Left autonomous\n");
        // Mirrored version of red right
        redLeft();  // Placeholder
    }
    
    void blueRight() {
        printf("Running Blue Right autonomous\n");
        // Mirrored version of red left
        redLeft();  // Placeholder
    }
    
    void skills() {
        printf("Running Skills autonomous\n");
        
        // Skills autonomous is typically longer and more complex
        // Would load from instruction file
        
        std::vector<Instruction> instructions = loadInstructions("/usd/instructions/skills_auto.txt");
        if (!instructions.empty()) {
            executeInstructions(instructions);
        } else {
            printf("No skills instructions found, using fallback\n");
            // Fallback simple skills routine
        }
    }
    
    void test() {
        printf("Running Test autonomous\n");
        
        // Test basic movements
        printf("Testing drive forward...\n");
        driveDistance(24, 40);
        pros::delay(500);
        
        printf("Testing turn...\n");
        turnToHeadingDeg(90, 3000);
        pros::delay(500);
        
        printf("Testing drive backward...\n");
        driveDistance(-24, 40);
        
        printf("Test complete\n");
    }
}

