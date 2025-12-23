/**
 * @file physics_simulator.cpp
 * @brief Physics-based VEX robot simulator with PID testing
 * 
 * Compile with: clang++ -std=c++17 physics_simulator.cpp -o physics_simulator `sdl2-config --cflags --libs`
 * Run with: ./physics_simulator
 */

#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <cstring>
#include <algorithm>

// Field dimensions (in inches)
const double FIELD_WIDTH = 144.0;  // 12 feet
const double FIELD_HEIGHT = 144.0; // 12 feet
const double PIXELS_PER_INCH = 4.0; // Scale factor

// Robot physical properties
const double ROBOT_WIDTH = 18.0;   // inches
const double ROBOT_HEIGHT = 18.0;  // inches
const double ROBOT_MASS = 10.0;     // kg (approximate)
const double WHEEL_BASE = 12.5;    // inches (distance between left/right wheels)
const double WHEEL_RADIUS = 1.625;  // inches (3.25" diameter / 2)
const double MAX_MOTOR_TORQUE = 2.1; // N⋅m (V5 motor max)
const double FRICTION_COEFFICIENT = 0.3; // Rolling friction
const double MOMENT_OF_INERTIA = 0.5; // kg⋅m² (rotational inertia)

// PID Controller (standalone, no PROS dependencies)
class PIDController {
private:
    double m_kP, m_kI, m_kD;
    double m_integral = 0;
    double m_lastError = 0;
    double m_maxIntegral = 50.0;
    double m_minOutput = -100;
    double m_maxOutput = 100;
    bool m_hasOutputLimits = true;
    double m_lastTime = 0;
    
    double m_pTerm = 0;
    double m_iTerm = 0;
    double m_dTerm = 0;
    
public:
    PIDController(double kP, double kI, double kD, double maxIntegral = 50.0)
        : m_kP(kP), m_kI(kI), m_kD(kD), m_maxIntegral(maxIntegral) {}
    
    double calculate(double target, double current, double dt) {
        double error = target - current;
        return calculateFromError(error, dt);
    }
    
    double calculateFromError(double error, double dt) {
        if (dt <= 0) dt = 0.02; // Default 20ms
        
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
        
        // Apply output limits
        if (m_hasOutputLimits) {
            output = std::max(m_minOutput, std::min(m_maxOutput, output));
        }
        
        return output;
    }
    
    void reset() {
        m_integral = 0;
        m_lastError = 0;
        m_pTerm = m_iTerm = m_dTerm = 0;
    }
    
    void setGains(double kP, double kI, double kD) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
    }
    
    double getP() const { return m_pTerm; }
    double getI() const { return m_iTerm; }
    double getD() const { return m_dTerm; }
    double getError() const { return m_lastError; }
};

struct RobotPhysics {
    // Position and orientation
    double x = 72.0;        // inches
    double y = 72.0;
    double heading = 0.0;   // radians
    
    // Linear and angular velocity
    double vx = 0.0;        // inches/second
    double vy = 0.0;
    double angularVel = 0.0; // radians/second
    
    // Forces and torques
    double leftMotorForce = 0.0;   // Newtons
    double rightMotorForce = 0.0;
    
    // Encoders (simulated)
    double leftEncoder = 0.0;   // degrees
    double rightEncoder = 0.0;
    double verticalEncoder = 0.0;
    double horizontalEncoder = 0.0;
    
    // IMU (simulated)
    double imuHeading = 0.0;  // degrees
};

struct Waypoint {
    double x, y;
    bool reached = false;
};

enum class ControlMode {
    MANUAL,      // Direct keyboard control
    DRIVE_TO,    // PID drive to point
    TURN_TO,     // PID turn to heading
    FOLLOW_PATH  // Follow waypoints
};

class PhysicsSimulator {
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    bool running;
    RobotPhysics robot;
    std::vector<Waypoint> waypoints;
    
    // PID Controllers
    PIDController linearPID;   // For driving distance
    PIDController angularPID;  // For heading correction
    PIDController turnPID;     // For point turns
    
    // Control state
    ControlMode controlMode = ControlMode::MANUAL;
    double targetX = 0;
    double targetY = 0;
    double targetHeading = 0;
    int currentWaypoint = -1;
    
    // Display options
    bool showOdometry = true;
    bool showPIDInfo = true;
    bool showForces = false;
    bool showTrajectory = true;
    std::vector<std::pair<double, double>> trajectory;
    
    // PID tuning (adjustable) - initialized first
    double linearKP = 1.0, linearKI = 0.0, linearKD = 0.1;
    double angularKP = 2.0, angularKI = 0.0, angularKD = 0.2;
    double turnKP = 2.5, turnKI = 0.0, turnKD = 0.3;
    
    // Colors
    SDL_Color fieldColor = {240, 240, 240, 255};
    SDL_Color robotColor = {50, 150, 255, 255};
    SDL_Color waypointColor = {255, 100, 100, 255};
    SDL_Color targetColor = {255, 200, 0, 255};
    SDL_Color trajectoryColor = {200, 200, 255, 255};
    SDL_Color forceColor = {255, 0, 0, 255};
    
    double lastTime = 0;
    
public:
    PhysicsSimulator() 
        : linearPID(1.0, 0.0, 0.1),
          angularPID(2.0, 0.0, 0.2),
          turnPID(2.5, 0.0, 0.3) {
        
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
            exit(1);
        }
        
        int width = static_cast<int>(FIELD_WIDTH * PIXELS_PER_INCH);
        int height = static_cast<int>(FIELD_HEIGHT * PIXELS_PER_INCH);
        
        window = SDL_CreateWindow(
            "VEX Robot Physics Simulator - PID Testing",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            width + 350,
            height,
            SDL_WINDOW_SHOWN
        );
        
        if (!window) {
            std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
            exit(1);
        }
        
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if (!renderer) {
            std::cerr << "Renderer creation failed: " << SDL_GetError() << std::endl;
            exit(1);
        }
    }
    
    ~PhysicsSimulator() {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
    
    void handleEvents() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                    
                case SDL_MOUSEBUTTONDOWN:
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        int mouseX = event.button.x;
                        int mouseY = event.button.y;
                        
                        if (mouseX < FIELD_WIDTH * PIXELS_PER_INCH) {
                            double fieldX = mouseX / PIXELS_PER_INCH;
                            double fieldY = mouseY / PIXELS_PER_INCH;
                            
                            if (controlMode == ControlMode::DRIVE_TO) {
                                targetX = fieldX;
                                targetY = fieldY;
                                linearPID.reset();
                                angularPID.reset();
                                std::cout << "Target set to (" << targetX << ", " << targetY << ")\n";
                            } else {
                                Waypoint wp;
                                wp.x = fieldX;
                                wp.y = fieldY;
                                waypoints.push_back(wp);
                                std::cout << "Added waypoint at (" << fieldX << ", " << fieldY << ")\n";
                            }
                        }
                    } else if (event.button.button == SDL_BUTTON_RIGHT) {
                        int mouseX = event.button.x;
                        int mouseY = event.button.y;
                        
                        if (mouseX < FIELD_WIDTH * PIXELS_PER_INCH) {
                            robot.x = mouseX / PIXELS_PER_INCH;
                            robot.y = mouseY / PIXELS_PER_INCH;
                            robot.vx = robot.vy = robot.angularVel = 0;
                            trajectory.clear();
                            std::cout << "Reset robot position\n";
                        }
                    }
                    break;
                    
                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym) {
                        case SDLK_r:
                            robot.x = robot.y = 72.0;
                            robot.heading = 0;
                            robot.vx = robot.vy = robot.angularVel = 0;
                            waypoints.clear();
                            trajectory.clear();
                            linearPID.reset();
                            angularPID.reset();
                            turnPID.reset();
                            break;
                        case SDLK_m:
                            controlMode = ControlMode::MANUAL;
                            std::cout << "Mode: MANUAL\n";
                            break;
                        case SDLK_d:
                            controlMode = ControlMode::DRIVE_TO;
                            std::cout << "Mode: DRIVE_TO (click to set target)\n";
                            break;
                        case SDLK_t:
                            controlMode = ControlMode::TURN_TO;
                            targetHeading = robot.heading * 180.0 / M_PI + 90;
                            turnPID.reset();
                            std::cout << "Mode: TURN_TO (target: " << targetHeading << "°)\n";
                            break;
                        case SDLK_f:
                            controlMode = ControlMode::FOLLOW_PATH;
                            currentWaypoint = 0;
                            if (!waypoints.empty()) {
                                targetX = waypoints[0].x;
                                targetY = waypoints[0].y;
                            }
                            std::cout << "Mode: FOLLOW_PATH\n";
                            break;
                        case SDLK_o:
                            showOdometry = !showOdometry;
                            break;
                        case SDLK_p:
                            showPIDInfo = !showPIDInfo;
                            break;
                        case SDLK_w:
                            showForces = !showForces;
                            break;
                        case SDLK_g:
                            showTrajectory = !showTrajectory;
                            break;
                        case SDLK_UP:
                            if (controlMode == ControlMode::MANUAL) {
                                robot.leftMotorForce += 5;
                                robot.rightMotorForce += 5;
                            } else if (controlMode == ControlMode::TURN_TO) {
                                targetHeading += 5;
                            }
                            break;
                        case SDLK_DOWN:
                            if (controlMode == ControlMode::MANUAL) {
                                robot.leftMotorForce -= 5;
                                robot.rightMotorForce -= 5;
                            } else if (controlMode == ControlMode::TURN_TO) {
                                targetHeading -= 5;
                            }
                            break;
                        case SDLK_LEFT:
                            if (controlMode == ControlMode::MANUAL) {
                                robot.leftMotorForce -= 5;
                                robot.rightMotorForce += 5;
                            }
                            break;
                        case SDLK_RIGHT:
                            if (controlMode == ControlMode::MANUAL) {
                                robot.leftMotorForce += 5;
                                robot.rightMotorForce -= 5;
                            }
                            break;
                        case SDLK_SPACE:
                            robot.leftMotorForce = robot.rightMotorForce = 0;
                            break;
                        case SDLK_1:
                            linearKP = std::max(0.0, linearKP - 0.1);
                            linearPID.setGains(linearKP, linearKI, linearKD);
                            std::cout << "Linear KP: " << linearKP << "\n";
                            break;
                        case SDLK_2:
                            linearKP = std::min(10.0, linearKP + 0.1);
                            linearPID.setGains(linearKP, linearKI, linearKD);
                            std::cout << "Linear KP: " << linearKP << "\n";
                            break;
                        case SDLK_3:
                            linearKI = std::max(0.0, linearKI - 0.01);
                            linearPID.setGains(linearKP, linearKI, linearKD);
                            std::cout << "Linear KI: " << linearKI << "\n";
                            break;
                        case SDLK_4:
                            linearKI = std::min(1.0, linearKI + 0.01);
                            linearPID.setGains(linearKP, linearKI, linearKD);
                            std::cout << "Linear KI: " << linearKI << "\n";
                            break;
                        case SDLK_5:
                            linearKD = std::max(0.0, linearKD - 0.01);
                            linearPID.setGains(linearKP, linearKI, linearKD);
                            std::cout << "Linear KD: " << linearKD << "\n";
                            break;
                        case SDLK_6:
                            linearKD = std::min(1.0, linearKD + 0.01);
                            linearPID.setGains(linearKP, linearKI, linearKD);
                            std::cout << "Linear KD: " << linearKD << "\n";
                            break;
                        case SDLK_c:
                            waypoints.clear();
                            currentWaypoint = -1;
                            break;
                    }
                    break;
            }
        }
    }
    
    void updatePhysics(double dt) {
        // Limit forces to motor capabilities
        robot.leftMotorForce = std::max(-MAX_MOTOR_TORQUE, std::min(MAX_MOTOR_TORQUE, robot.leftMotorForce));
        robot.rightMotorForce = std::max(-MAX_MOTOR_TORQUE, std::min(MAX_MOTOR_TORQUE, robot.rightMotorForce));
        
        // Convert motor forces to linear and angular acceleration
        double totalForce = (robot.leftMotorForce + robot.rightMotorForce) / WHEEL_RADIUS;
        double torque = (robot.rightMotorForce - robot.leftMotorForce) * WHEEL_BASE / (2.0 * WHEEL_RADIUS);
        
        // Apply friction
        double frictionForce = -FRICTION_COEFFICIENT * ROBOT_MASS * 9.81 * 0.0254; // Convert to Newtons
        double frictionTorque = -0.1 * robot.angularVel; // Angular friction
        
        // Calculate accelerations
        double ax = (totalForce * cos(robot.heading) + frictionForce * cos(robot.heading)) / ROBOT_MASS;
        double ay = (totalForce * sin(robot.heading) + frictionForce * sin(robot.heading)) / ROBOT_MASS;
        double alpha = (torque + frictionTorque) / MOMENT_OF_INERTIA;
        
        // Update velocities
        robot.vx += ax * dt;
        robot.vy += ay * dt;
        robot.angularVel += alpha * dt;
        
        // Apply velocity damping
        robot.vx *= 0.98;
        robot.vy *= 0.98;
        robot.angularVel *= 0.95;
        
        // Update position
        robot.x += robot.vx * dt * 0.0254; // Convert m/s to in/s
        robot.y += robot.vy * dt * 0.0254;
        robot.heading += robot.angularVel * dt;
        
        // Normalize heading
        while (robot.heading > 2 * M_PI) robot.heading -= 2 * M_PI;
        while (robot.heading < 0) robot.heading += 2 * M_PI;
        
        // Update sensors
        double linearVel = sqrt(robot.vx * robot.vx + robot.vy * robot.vy) / 0.0254; // in/s
        robot.leftEncoder += (linearVel - robot.angularVel * WHEEL_BASE / 2.0) * dt * 360.0 / (2.0 * M_PI * WHEEL_RADIUS);
        robot.rightEncoder += (linearVel + robot.angularVel * WHEEL_BASE / 2.0) * dt * 360.0 / (2.0 * M_PI * WHEEL_RADIUS);
        robot.verticalEncoder += linearVel * dt * 360.0 / (2.0 * M_PI * WHEEL_RADIUS);
        robot.imuHeading = robot.heading * 180.0 / M_PI;
        
        // Record trajectory
        if (showTrajectory) {
            trajectory.push_back({robot.x, robot.y});
            if (trajectory.size() > 1000) trajectory.erase(trajectory.begin());
        }
    }
    
    void updateControl(double dt) {
        if (controlMode == ControlMode::DRIVE_TO) {
            // Calculate distance and angle to target
            double dx = targetX - robot.x;
            double dy = targetY - robot.y;
            double distance = sqrt(dx * dx + dy * dy);
            double targetAngle = atan2(dy, dx);
            
            // Angle error (normalize to [-PI, PI])
            double angleError = targetAngle - robot.heading;
            while (angleError > M_PI) angleError -= 2 * M_PI;
            while (angleError < -M_PI) angleError += 2 * M_PI;
            
            // Use angular PID to correct heading
            double angularOutput = angularPID.calculateFromError(angleError, dt);
            
            // Use linear PID for distance
            double linearOutput = linearPID.calculate(0, distance, dt);
            
            // Convert to motor forces
            double baseForce = linearOutput * 0.1; // Scale
            double turnForce = angularOutput * 0.1;
            
            robot.leftMotorForce = baseForce - turnForce;
            robot.rightMotorForce = baseForce + turnForce;
            
            // Check if reached
            if (distance < 2.0) {
                robot.leftMotorForce = robot.rightMotorForce = 0;
            }
            
        } else if (controlMode == ControlMode::TURN_TO) {
            double targetRad = targetHeading * M_PI / 180.0;
            double angleError = targetRad - robot.heading;
            while (angleError > M_PI) angleError -= 2 * M_PI;
            while (angleError < -M_PI) angleError += 2 * M_PI;
            
            double output = turnPID.calculateFromError(angleError, dt);
            
            robot.leftMotorForce = -output * 0.1;
            robot.rightMotorForce = output * 0.1;
            
            if (abs(angleError) < 0.05) { // ~3 degrees
                robot.leftMotorForce = robot.rightMotorForce = 0;
            }
            
        } else if (controlMode == ControlMode::FOLLOW_PATH) {
            if (currentWaypoint >= 0 && currentWaypoint < waypoints.size()) {
                targetX = waypoints[currentWaypoint].x;
                targetY = waypoints[currentWaypoint].y;
                
                double dx = targetX - robot.x;
                double dy = targetY - robot.y;
                double distance = sqrt(dx * dx + dy * dy);
                
                if (distance < 3.0) {
                    waypoints[currentWaypoint].reached = true;
                    currentWaypoint++;
                    if (currentWaypoint >= waypoints.size()) {
                        currentWaypoint = -1;
                        robot.leftMotorForce = robot.rightMotorForce = 0;
                    }
                } else {
                    // Same as DRIVE_TO
                    double targetAngle = atan2(dy, dx);
                    double angleError = targetAngle - robot.heading;
                    while (angleError > M_PI) angleError -= 2 * M_PI;
                    while (angleError < -M_PI) angleError += 2 * M_PI;
                    
                    double angularOutput = angularPID.calculateFromError(angleError, dt);
                    double linearOutput = linearPID.calculate(0, distance, dt);
                    
                    double baseForce = linearOutput * 0.1;
                    double turnForce = angularOutput * 0.1;
                    
                    robot.leftMotorForce = baseForce - turnForce;
                    robot.rightMotorForce = baseForce + turnForce;
                }
            }
        }
    }
    
    void update(double dt) {
        updateControl(dt);
        updatePhysics(dt);
    }
    
    void drawField() {
        SDL_SetRenderDrawColor(renderer, fieldColor.r, fieldColor.g, fieldColor.b, fieldColor.a);
        SDL_Rect field = {0, 0, 
                         static_cast<int>(FIELD_WIDTH * PIXELS_PER_INCH),
                         static_cast<int>(FIELD_HEIGHT * PIXELS_PER_INCH)};
        SDL_RenderFillRect(renderer, &field);
        
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
        for (int i = 0; i <= 12; i++) {
            int x = static_cast<int>(i * 12 * PIXELS_PER_INCH);
            SDL_RenderDrawLine(renderer, x, 0, x, static_cast<int>(FIELD_HEIGHT * PIXELS_PER_INCH));
            int y = static_cast<int>(i * 12 * PIXELS_PER_INCH);
            SDL_RenderDrawLine(renderer, 0, y, static_cast<int>(FIELD_WIDTH * PIXELS_PER_INCH), y);
        }
    }
    
    void drawTrajectory() {
        if (!showTrajectory || trajectory.size() < 2) return;
        
        SDL_SetRenderDrawColor(renderer, trajectoryColor.r, trajectoryColor.g, trajectoryColor.b, 100);
        for (size_t i = 1; i < trajectory.size(); i++) {
            int x1 = static_cast<int>(trajectory[i-1].first * PIXELS_PER_INCH);
            int y1 = static_cast<int>(trajectory[i-1].second * PIXELS_PER_INCH);
            int x2 = static_cast<int>(trajectory[i].first * PIXELS_PER_INCH);
            int y2 = static_cast<int>(trajectory[i].second * PIXELS_PER_INCH);
            SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        }
    }
    
    void drawRobot() {
        int centerX = static_cast<int>(robot.x * PIXELS_PER_INCH);
        int centerY = static_cast<int>(robot.y * PIXELS_PER_INCH);
        
        // Draw robot body
        SDL_SetRenderDrawColor(renderer, robotColor.r, robotColor.g, robotColor.b, robotColor.a);
        
        double cos_h = cos(robot.heading);
        double sin_h = sin(robot.heading);
        double w2 = ROBOT_WIDTH * PIXELS_PER_INCH / 2.0;
        double h2 = ROBOT_HEIGHT * PIXELS_PER_INCH / 2.0;
        
        SDL_Point corners[5];
        corners[0] = {static_cast<int>(centerX + (w2 * cos_h - h2 * sin_h)),
                      static_cast<int>(centerY + (w2 * sin_h + h2 * cos_h))};
        corners[1] = {static_cast<int>(centerX + (-w2 * cos_h - h2 * sin_h)),
                      static_cast<int>(centerY + (-w2 * sin_h + h2 * cos_h))};
        corners[2] = {static_cast<int>(centerX + (-w2 * cos_h + h2 * sin_h)),
                      static_cast<int>(centerY + (-w2 * sin_h - h2 * cos_h))};
        corners[3] = {static_cast<int>(centerX + (w2 * cos_h + h2 * sin_h)),
                      static_cast<int>(centerY + (w2 * sin_h - h2 * cos_h))};
        corners[4] = corners[0];
        
        SDL_RenderDrawLines(renderer, corners, 5);
        
        // Draw heading arrow
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        int arrowLen = static_cast<int>(ROBOT_WIDTH * PIXELS_PER_INCH / 2.0);
        SDL_RenderDrawLine(renderer, centerX, centerY,
                          static_cast<int>(centerX + arrowLen * cos_h),
                          static_cast<int>(centerY + arrowLen * sin_h));
        
        // Draw forces if enabled
        if (showForces) {
            SDL_SetRenderDrawColor(renderer, forceColor.r, forceColor.g, forceColor.b, forceColor.a);
            int forceScale = 20;
            SDL_RenderDrawLine(renderer, centerX, centerY,
                              static_cast<int>(centerX + robot.leftMotorForce * forceScale * cos(robot.heading - M_PI/2)),
                              static_cast<int>(centerY + robot.leftMotorForce * forceScale * sin(robot.heading - M_PI/2)));
            SDL_RenderDrawLine(renderer, centerX, centerY,
                              static_cast<int>(centerX + robot.rightMotorForce * forceScale * cos(robot.heading + M_PI/2)),
                              static_cast<int>(centerY + robot.rightMotorForce * forceScale * sin(robot.heading + M_PI/2)));
        }
    }
    
    void drawTarget() {
        if (controlMode == ControlMode::DRIVE_TO || controlMode == ControlMode::FOLLOW_PATH) {
            int x = static_cast<int>(targetX * PIXELS_PER_INCH);
            int y = static_cast<int>(targetY * PIXELS_PER_INCH);
            
            SDL_SetRenderDrawColor(renderer, targetColor.r, targetColor.g, targetColor.b, 255);
            for (int angle = 0; angle < 360; angle++) {
                double rad = angle * M_PI / 180.0;
                int px = x + static_cast<int>(10 * cos(rad));
                int py = y + static_cast<int>(10 * sin(rad));
                SDL_RenderDrawPoint(renderer, px, py);
            }
        }
    }
    
    void drawWaypoints() {
        for (const auto& wp : waypoints) {
            int x = static_cast<int>(wp.x * PIXELS_PER_INCH);
            int y = static_cast<int>(wp.y * PIXELS_PER_INCH);
            
            SDL_SetRenderDrawColor(renderer, 
                                 wp.reached ? 100 : waypointColor.r,
                                 wp.reached ? 255 : waypointColor.g,
                                 wp.reached ? 100 : waypointColor.b,
                                 255);
            
            for (int angle = 0; angle < 360; angle++) {
                double rad = angle * M_PI / 180.0;
                int px = x + static_cast<int>(8 * cos(rad));
                int py = y + static_cast<int>(8 * sin(rad));
                SDL_RenderDrawPoint(renderer, px, py);
            }
        }
    }
    
    void drawInfoPanel() {
        int panelX = static_cast<int>(FIELD_WIDTH * PIXELS_PER_INCH) + 10;
        
        SDL_SetRenderDrawColor(renderer, 250, 250, 250, 255);
        SDL_Rect panel = {panelX, 0, 340, 576};
        SDL_RenderFillRect(renderer, &panel);
        
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderDrawRect(renderer, &panel);
        
        // Print info to console (SDL doesn't have built-in text rendering)
        static int printCounter = 0;
        if (printCounter++ % 60 == 0) { // Every second
            std::cout << "\r";
            std::cout << "Pos: (" << (int)robot.x << ", " << (int)robot.y << ") ";
            std::cout << "H: " << (int)(robot.heading * 180.0 / M_PI) << "° ";
            std::cout << "Vel: " << (int)sqrt(robot.vx*robot.vx + robot.vy*robot.vy) << " in/s";
            if (showPIDInfo) {
                std::cout << " | PID P:" << (int)linearPID.getP() << " I:" << (int)linearPID.getI() << " D:" << (int)linearPID.getD();
            }
            std::cout << std::flush;
        }
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);
        
        drawField();
        drawTrajectory();
        drawWaypoints();
        drawTarget();
        drawRobot();
        drawInfoPanel();
        
        SDL_RenderPresent(renderer);
    }
    
    void run() {
        std::cout << "\n=== VEX Robot Physics Simulator with PID ===\n";
        std::cout << "Controls:\n";
        std::cout << "  M: Manual mode (arrow keys)\n";
        std::cout << "  D: Drive-to mode (click target)\n";
        std::cout << "  T: Turn-to mode\n";
        std::cout << "  F: Follow path (waypoints)\n";
        std::cout << "  Left Click: Add waypoint / Set target\n";
        std::cout << "  Right Click: Reset robot position\n";
        std::cout << "  R: Reset everything\n";
        std::cout << "  P: Toggle PID info\n";
        std::cout << "  W: Toggle force vectors\n";
        std::cout << "  G: Toggle trajectory\n";
        std::cout << "  1/2: Adjust Linear KP\n";
        std::cout << "  3/4: Adjust Linear KI\n";
        std::cout << "  5/6: Adjust Linear KD\n";
        std::cout << "\n";
        
        Uint32 lastTime = SDL_GetTicks();
        
        while (running) {
            handleEvents();
            
            Uint32 currentTime = SDL_GetTicks();
            double dt = (currentTime - lastTime) / 1000.0;
            lastTime = currentTime;
            
            if (dt > 0.1) dt = 0.1;
            
            update(dt);
            render();
            
            SDL_Delay(16); // ~60 FPS
        }
        
        std::cout << "\n";
    }
};

int main(int argc, char* argv[]) {
    PhysicsSimulator simulator;
    simulator.run();
    return 0;
}

