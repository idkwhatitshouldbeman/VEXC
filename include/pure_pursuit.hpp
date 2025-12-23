/**
 * @file pure_pursuit.hpp
 * @brief Pure Pursuit path following algorithm
 */

#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "api.h"
#include "odometry.hpp"
#include <vector>
#include <string>

/**
 * @struct Waypoint
 * @brief A single point on a path
 */
struct Waypoint {
    double x;
    double y;
    double velocity;    // Target velocity at this point (0-1 normalized, or 0-120 from jerryio)
    
    Waypoint() : x(0), y(0), velocity(1.0) {}
    Waypoint(double _x, double _y, double _vel = 1.0) : x(_x), y(_y), velocity(_vel) {}
};

/**
 * @struct Path
 * @brief A complete path with multiple waypoints
 */
struct Path {
    std::vector<Waypoint> waypoints;
    double lookaheadDistance;
    double tolerance;
    bool reverse;               // Drive backwards
    bool decelerate;            // Slow down near end
    double decelerateDistance;  // Distance to start decelerating
    double decelerateRate;      // Deceleration factor
    double lastPointTolerance;  // Tolerance for final point
    
    Path() : lookaheadDistance(50.0), tolerance(6.0), reverse(false),
             decelerate(false), decelerateDistance(22.0), decelerateRate(0.6),
             lastPointTolerance(2.5) {}
    
    /**
     * @brief Check if path is empty
     */
    bool empty() const { return waypoints.empty(); }
    
    /**
     * @brief Get number of waypoints
     */
    size_t size() const { return waypoints.size(); }
    
    /**
     * @brief Clear all waypoints
     */
    void clear() { waypoints.clear(); }
    
    /**
     * @brief Add a waypoint
     */
    void addWaypoint(double x, double y, double vel = 1.0) {
        waypoints.emplace_back(x, y, vel);
    }
    
    /**
     * @brief Get the last waypoint
     */
    const Waypoint& back() const { return waypoints.back(); }
};

/**
 * @class PurePursuit
 * @brief Pure pursuit path following controller
 */
class PurePursuit {
public:
    /**
     * @brief Follow a path to completion
     * @param path The path to follow
     * @param direction 1 for forward, -1 for reverse
     */
    static void followPath(Path& path, int direction = 1);
    
    /**
     * @brief Load a path from a Jerry.io CSV file
     * @param filename Path to the CSV file on SD card
     * @return Loaded path
     */
    static Path loadPathFromCSV(const std::string& filename);
    
    /**
     * @brief Load a path from inline waypoints (like in reference code)
     * @param points Vector of (x, y) or (x, y, velocity) tuples
     * @return Loaded path
     */
    static Path createPath(const std::vector<std::pair<double, double>>& points);
    
    /**
     * @brief Create a mirrored path (for opposite alliance)
     * @param path Original path
     * @param mirrorX Mirror across X axis
     * @param mirrorY Mirror across Y axis
     * @return Mirrored path
     */
    static Path mirrorPath(const Path& path, bool mirrorX = false, bool mirrorY = false);
    
    /**
     * @brief Stop following the current path
     */
    static void stop();
    
    /**
     * @brief Check if currently following a path
     */
    static bool isFollowing();
    
    /**
     * @brief Set the forward velocity
     */
    static void setForwardVelocity(double velocity);
    
    /**
     * @brief Set the turn velocity constant
     */
    static void setTurnVelocityK(double k);
    
    /**
     * @brief Get the current forward velocity setting
     */
    static double getForwardVelocity();
    
    /**
     * @brief Get the current turn velocity K
     */
    static double getTurnVelocityK();

private:
    /**
     * @brief Find the lookahead point on the path
     */
    static Waypoint findLookaheadPoint(Path& path, const RobotPosition& pos, double lookaheadDist);
    
    /**
     * @brief Calculate drive speeds for differential drive
     */
    static void calculateDriveSpeeds(const Waypoint& lookahead, int direction,
                                     double& leftVel, double& rightVel);
    
    /**
     * @brief Calculate curvature to reach a point
     */
    static double calculateCurvature(const RobotPosition& pos, const Waypoint& target);
    
    // State
    static bool s_following;
    static double s_forwardVelocity;
    static double s_turnVelocityK;
    static double s_leftVelocity;
    static double s_rightVelocity;
};

#endif // PURE_PURSUIT_HPP

