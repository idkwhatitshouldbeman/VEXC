/**
 * @file visual_simulator.cpp
 * @brief Visual simulator for VEX robot with GUI
 * 
 * Compile with: clang++ -std=c++17 visual_simulator.cpp -o visual_simulator `sdl2-config --cflags --libs`
 * Run with: ./visual_simulator
 */

#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <cstring>

// Field dimensions (in inches)
const double FIELD_WIDTH = 144.0;  // 12 feet
const double FIELD_HEIGHT = 144.0; // 12 feet
const double PIXELS_PER_INCH = 4.0; // Scale factor

// Robot dimensions
const double ROBOT_WIDTH = 18.0;  // inches
const double ROBOT_HEIGHT = 18.0;  // inches

struct RobotState {
    double x = 72.0;        // Start in center (inches)
    double y = 72.0;
    double heading = 0.0;   // radians
    double leftVel = 0.0;
    double rightVel = 0.0;
    
    // Odometry
    double odomX = 72.0;
    double odomY = 72.0;
    double odomHeading = 0.0;
    
    // Sensors
    double imuHeading = 0.0;
    double leftEncoder = 0.0;
    double rightEncoder = 0.0;
    double verticalEncoder = 0.0;
    double horizontalEncoder = 0.0;
};

struct Waypoint {
    double x, y;
    bool reached = false;
};

class VisualSimulator {
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    bool running;
    RobotState robot;
    std::vector<Waypoint> waypoints;
    int selectedWaypoint = -1;
    bool showOdometry = true;
    bool showSensors = true;
    
    // Colors
    SDL_Color fieldColor = {240, 240, 240, 255};
    SDL_Color robotColor = {50, 150, 255, 255};
    SDL_Color waypointColor = {255, 100, 100, 255};
    SDL_Color odomColor = {100, 255, 100, 255};
    SDL_Color textColor = {0, 0, 0, 255};
    
public:
    VisualSimulator() : running(true) {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
            exit(1);
        }
        
        int width = static_cast<int>(FIELD_WIDTH * PIXELS_PER_INCH);
        int height = static_cast<int>(FIELD_HEIGHT * PIXELS_PER_INCH);
        
        window = SDL_CreateWindow(
            "VEX Robot Simulator",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            width + 300, // Extra space for info panel
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
    
    ~VisualSimulator() {
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
                        
                        // Check if click is in field area
                        if (mouseX < FIELD_WIDTH * PIXELS_PER_INCH) {
                            double fieldX = mouseX / PIXELS_PER_INCH;
                            double fieldY = mouseY / PIXELS_PER_INCH;
                            
                            // Add waypoint
                            Waypoint wp;
                            wp.x = fieldX;
                            wp.y = fieldY;
                            waypoints.push_back(wp);
                            std::cout << "Added waypoint at (" << fieldX << ", " << fieldY << ")\n";
                        }
                    } else if (event.button.button == SDL_BUTTON_RIGHT) {
                        // Right click to set robot position
                        int mouseX = event.button.x;
                        int mouseY = event.button.y;
                        
                        if (mouseX < FIELD_WIDTH * PIXELS_PER_INCH) {
                            robot.x = mouseX / PIXELS_PER_INCH;
                            robot.y = mouseY / PIXELS_PER_INCH;
                            robot.odomX = robot.x;
                            robot.odomY = robot.y;
                            std::cout << "Set robot position to (" << robot.x << ", " << robot.y << ")\n";
                        }
                    }
                    break;
                    
                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym) {
                        case SDLK_r:
                            // Reset robot to center
                            robot.x = robot.y = 72.0;
                            robot.heading = robot.odomHeading = 0.0;
                            robot.odomX = robot.odomY = 72.0;
                            waypoints.clear();
                            break;
                        case SDLK_o:
                            showOdometry = !showOdometry;
                            break;
                        case SDLK_s:
                            showSensors = !showSensors;
                            break;
                        case SDLK_c:
                            waypoints.clear();
                            break;
                        case SDLK_UP:
                            robot.leftVel += 10;
                            robot.rightVel += 10;
                            break;
                        case SDLK_DOWN:
                            robot.leftVel -= 10;
                            robot.rightVel -= 10;
                            break;
                        case SDLK_LEFT:
                            robot.leftVel -= 10;
                            robot.rightVel += 10;
                            break;
                        case SDLK_RIGHT:
                            robot.leftVel += 10;
                            robot.rightVel -= 10;
                            break;
                        case SDLK_SPACE:
                            robot.leftVel = robot.rightVel = 0;
                            break;
                    }
                    break;
            }
        }
    }
    
    void update(double dt) {
        // Simulate robot movement
        const double WHEEL_BASE = 12.5; // inches
        const double MAX_VEL = 600.0; // RPM
        
        // Convert RPM to inches per second (assuming 3.25" wheels)
        double leftSpeed = (robot.leftVel / MAX_VEL) * (3.25 * M_PI * 600.0 / 60.0);
        double rightSpeed = (robot.rightVel / MAX_VEL) * (3.25 * M_PI * 600.0 / 60.0);
        
        // Calculate linear and angular velocity
        double linearVel = (leftSpeed + rightSpeed) / 2.0;
        double angularVel = (rightSpeed - leftSpeed) / WHEEL_BASE;
        
        // Update position
        robot.x += linearVel * cos(robot.heading) * dt;
        robot.y += linearVel * sin(robot.heading) * dt;
        robot.heading += angularVel * dt;
        
        // Normalize heading
        while (robot.heading > 2 * M_PI) robot.heading -= 2 * M_PI;
        while (robot.heading < 0) robot.heading += 2 * M_PI;
        
        // Update odometry (simplified)
        robot.odomX += linearVel * cos(robot.odomHeading) * dt;
        robot.odomY += linearVel * sin(robot.odomHeading) * dt;
        robot.odomHeading += angularVel * dt;
        while (robot.odomHeading > 2 * M_PI) robot.odomHeading -= 2 * M_PI;
        while (robot.odomHeading < 0) robot.odomHeading += 2 * M_PI;
        
        // Update sensors
        robot.imuHeading = robot.heading * 180.0 / M_PI;
        robot.leftEncoder += leftSpeed * dt * 360.0 / (3.25 * M_PI);
        robot.rightEncoder += rightSpeed * dt * 360.0 / (3.25 * M_PI);
        robot.verticalEncoder += linearVel * dt * 360.0 / (2.0 * M_PI);
        
        // Check waypoint reaching
        for (auto& wp : waypoints) {
            if (!wp.reached) {
                double dist = sqrt(pow(robot.x - wp.x, 2) + pow(robot.y - wp.y, 2));
                if (dist < 3.0) { // 3 inch tolerance
                    wp.reached = true;
                    std::cout << "Reached waypoint at (" << wp.x << ", " << wp.y << ")\n";
                }
            }
        }
    }
    
    void drawField() {
        // Draw field background
        SDL_SetRenderDrawColor(renderer, fieldColor.r, fieldColor.g, fieldColor.b, fieldColor.a);
        SDL_Rect field = {0, 0, 
                         static_cast<int>(FIELD_WIDTH * PIXELS_PER_INCH),
                         static_cast<int>(FIELD_HEIGHT * PIXELS_PER_INCH)};
        SDL_RenderFillRect(renderer, &field);
        
        // Draw grid lines
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
        for (int i = 0; i <= 12; i++) {
            int x = static_cast<int>(i * 12 * PIXELS_PER_INCH);
            SDL_RenderDrawLine(renderer, x, 0, x, static_cast<int>(FIELD_HEIGHT * PIXELS_PER_INCH));
            int y = static_cast<int>(i * 12 * PIXELS_PER_INCH);
            SDL_RenderDrawLine(renderer, 0, y, static_cast<int>(FIELD_WIDTH * PIXELS_PER_INCH), y);
        }
    }
    
    void drawRobot() {
        int centerX = static_cast<int>(robot.x * PIXELS_PER_INCH);
        int centerY = static_cast<int>(robot.y * PIXELS_PER_INCH);
        
        // Draw robot body (rectangle)
        SDL_SetRenderDrawColor(renderer, robotColor.r, robotColor.g, robotColor.b, robotColor.a);
        
        // Calculate rotated corners
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
        corners[4] = corners[0]; // Close the polygon
        
        SDL_RenderDrawLines(renderer, corners, 5);
        
        // Draw heading arrow
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        int arrowLen = static_cast<int>(ROBOT_WIDTH * PIXELS_PER_INCH / 2.0);
        SDL_RenderDrawLine(renderer, centerX, centerY,
                          static_cast<int>(centerX + arrowLen * cos_h),
                          static_cast<int>(centerY + arrowLen * sin_h));
    }
    
    void drawOdometry() {
        if (!showOdometry) return;
        
        int centerX = static_cast<int>(robot.odomX * PIXELS_PER_INCH);
        int centerY = static_cast<int>(robot.odomY * PIXELS_PER_INCH);
        
        // Draw odometry position (green circle)
        SDL_SetRenderDrawColor(renderer, odomColor.r, odomColor.g, odomColor.b, odomColor.a);
        for (int angle = 0; angle < 360; angle++) {
            double rad = angle * M_PI / 180.0;
            int x = centerX + static_cast<int>(5 * cos(rad));
            int y = centerY + static_cast<int>(5 * sin(rad));
            SDL_RenderDrawPoint(renderer, x, y);
        }
        
        // Draw odometry heading
        SDL_RenderDrawLine(renderer, centerX, centerY,
                          static_cast<int>(centerX + 15 * cos(robot.odomHeading)),
                          static_cast<int>(centerY + 15 * sin(robot.odomHeading)));
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
            
            // Draw waypoint circle
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
        
        // Draw info panel background
        SDL_SetRenderDrawColor(renderer, 250, 250, 250, 255);
        SDL_Rect panel = {panelX, 0, 290, 576};
        SDL_RenderFillRect(renderer, &panel);
        
        // Draw border
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderDrawRect(renderer, &panel);
        
        // Note: SDL doesn't have built-in text rendering
        // In a full implementation, you'd use SDL_ttf
        // For now, we'll print to console
    }
    
    void printInfo() {
        std::cout << "\r";
        std::cout << "Robot: (" << (int)robot.x << ", " << (int)robot.y << ") ";
        std::cout << "Heading: " << (int)(robot.heading * 180.0 / M_PI) << "° ";
        std::cout << "Vel: L=" << (int)robot.leftVel << " R=" << (int)robot.rightVel;
        std::cout << " | Odometry: (" << (int)robot.odomX << ", " << (int)robot.odomY << ")";
        std::cout << " | Waypoints: " << waypoints.size();
        std::cout << std::flush;
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);
        
        drawField();
        drawWaypoints();
        if (showOdometry) drawOdometry();
        drawRobot();
        drawInfoPanel();
        
        SDL_RenderPresent(renderer);
    }
    
    void run() {
        Uint32 lastTime = SDL_GetTicks();
        
        std::cout << "\n=== VEX Robot Visual Simulator ===\n";
        std::cout << "Controls:\n";
        std::cout << "  Left Click: Add waypoint\n";
        std::cout << "  Right Click: Set robot position\n";
        std::cout << "  Arrow Keys: Control robot\n";
        std::cout << "  Space: Stop robot\n";
        std::cout << "  R: Reset to center\n";
        std::cout << "  O: Toggle odometry display\n";
        std::cout << "  S: Toggle sensor display\n";
        std::cout << "  C: Clear waypoints\n";
        std::cout << "  ESC/Close: Exit\n\n";
        
        while (running) {
            handleEvents();
            
            Uint32 currentTime = SDL_GetTicks();
            double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
            lastTime = currentTime;
            
            if (dt > 0.1) dt = 0.1; // Cap delta time
            
            update(dt);
            render();
            printInfo();
            
            SDL_Delay(16); // ~60 FPS
        }
        
        std::cout << "\n";
    }
};

int main(int argc, char* argv[]) {
    VisualSimulator simulator;
    simulator.run();
    return 0;
}

