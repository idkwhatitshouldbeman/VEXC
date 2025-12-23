/**
 * @file pure_pursuit.cpp
 * @brief Pure Pursuit path following implementation
 */

#include "pure_pursuit.hpp"
#include "config.hpp"
#include "utils.hpp"
#include "mechanisms.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

//==========================================================================
// STATIC MEMBER INITIALIZATION
//==========================================================================

bool PurePursuit::s_following = false;
double PurePursuit::s_forwardVelocity = Config::DEFAULT_FORWARD_VELOCITY;
double PurePursuit::s_turnVelocityK = Config::DEFAULT_TURN_VELOCITY_K;
double PurePursuit::s_leftVelocity = 0;
double PurePursuit::s_rightVelocity = 0;

//==========================================================================
// PATH FOLLOWING
//==========================================================================

void PurePursuit::followPath(Path& path, int direction) {
    if (path.empty()) {
        printf("Error: Empty path\n");
        return;
    }
    
    s_following = true;
    int stationaryCount = 0;
    const int MAX_STATIONARY = 10;
    
    size_t originalSize = path.size();
    
    // If position is at origin with zero heading, assume uninitialized and use first point
    RobotPosition pos = Odometry::getPosition();
    if (std::abs(pos.x) < 0.001 && std::abs(pos.y) < 0.001 && std::abs(pos.heading) < 0.001) {
        Odometry::setPosition(path.waypoints[0].x, path.waypoints[0].y, 0);
    }
    
    while (s_following && !path.empty()) {
        // Update mechanisms
        HighScoring::update();
        ColorDetection::update();
        Intake::update();
        
        // Check if robot is stuck
        stationaryCount = Odometry::getStationaryCount();
        if (stationaryCount > MAX_STATIONARY) {
            printf("Robot stuck, ending path\n");
            break;
        }
        
        // Get current position
        pos = Odometry::getPosition();
        
        // Find lookahead point
        Waypoint lookahead = findLookaheadPoint(path, pos, path.lookaheadDistance);
        
        // Calculate drive speeds
        calculateDriveSpeeds(lookahead, direction, s_leftVelocity, s_rightVelocity);
        
        // Apply deceleration near end if enabled
        if (path.decelerate && !path.empty()) {
            double distToEnd = Utils::distance(pos.x, pos.y, 
                                               path.waypoints.back().x, 
                                               path.waypoints.back().y);
            
            if (distToEnd < path.decelerateDistance) {
                double decelerateFactor = path.decelerateRate;
                if (distToEnd < path.decelerateDistance / 2.0) {
                    decelerateFactor = 0.4;
                } else {
                    decelerateFactor = (distToEnd / (distToEnd + distToEnd / 2.0)) * path.decelerateRate;
                }
                
                s_leftVelocity *= decelerateFactor;
                s_rightVelocity *= decelerateFactor;
            }
        }
        
        // Check if reached the target point
        if (!path.empty()) {
            double distToPoint = Utils::distance(pos.x, pos.y,
                                                 path.waypoints[0].x,
                                                 path.waypoints[0].y);
            
            // Check final point
            if (path.size() == 1) {
                double finalDist = Utils::distance(pos.x, pos.y,
                                                   path.waypoints.back().x,
                                                   path.waypoints.back().y);
                if (finalDist < path.lastPointTolerance) {
                    break;
                }
            } else if (distToPoint < path.tolerance) {
                path.waypoints.erase(path.waypoints.begin());
            }
        }
        
        // Set motor velocities
        Hardware::leftDrive.move_velocity(s_leftVelocity * 6.0);  // Convert percent to RPM (600 * percent/100)
        Hardware::rightDrive.move_velocity(s_rightVelocity * 6.0);
        
        pros::delay(Config::AUTONOMOUS_LOOP_DELAY);
    }
    
    // Stop motors
    Hardware::leftDrive.move(0);
    Hardware::rightDrive.move(0);
    s_following = false;
}

Waypoint PurePursuit::findLookaheadPoint(Path& path, const RobotPosition& pos, double lookaheadDist) {
    double closestDistance = INFINITY;
    int closestIndex = -1;
    
    // Remove points we've passed
    int minIndex = -1;
    for (size_t i = 0; i < path.size() - 1; i++) {
        double dist = Utils::distance(pos.x, pos.y, path.waypoints[i].x, path.waypoints[i].y);
        if (dist < path.tolerance) {
            minIndex = i;
        }
    }
    
    if (minIndex > 0) {
        path.waypoints.erase(path.waypoints.begin(), path.waypoints.begin() + minIndex);
    }
    
    // Check if path is empty after erasure - return last valid waypoint if so
    if (path.empty()) {
        // Return a waypoint at current position (shouldn't happen, but safe fallback)
        Waypoint fallback;
        fallback.x = pos.x;
        fallback.y = pos.y;
        return fallback;
    }
    
    // Now safe to use path.waypoints[0] since we know path is not empty
    Waypoint lookaheadPoint = path.waypoints[0];
    
    // Find closest point and lookahead point
    Waypoint closestPoint = path.waypoints[0];
    double tempSmallestLookahead = INFINITY;
    
    for (size_t i = 0; i < path.size() - 1; i++) {
        const Waypoint& start = path.waypoints[i];
        const Waypoint& end = path.waypoints[i + 1];
        
        double segmentLength = Utils::distance(start.x, start.y, end.x, end.y);
        if (segmentLength == 0) continue;
        
        // Project current position onto segment
        double t = ((pos.x - start.x) * (end.x - start.x) + 
                    (pos.y - start.y) * (end.y - start.y)) / (segmentLength * segmentLength);
        t = Utils::clamp(t, 0.0, 1.0);
        
        double closestX = start.x + t * (end.x - start.x);
        double closestY = start.y + t * (end.y - start.y);
        
        double distance = Utils::distance(pos.x, pos.y, closestX, closestY);
        
        // Special case for last segment
        if (path.size() == 2 && distance < 2 * path.tolerance) {
            closestPoint = path.waypoints[1];
            path.waypoints.erase(path.waypoints.begin());
            break;
        }
        
        if (distance < closestDistance) {
            closestDistance = distance;
            closestIndex = i;
            closestPoint.x = closestX;
            closestPoint.y = closestY;
            closestPoint.velocity = start.velocity;
        }
        
        if (distance >= lookaheadDist) {
            if (distance < tempSmallestLookahead) {
                tempSmallestLookahead = distance;
                lookaheadPoint.x = closestX;
                lookaheadPoint.y = closestY;
                lookaheadPoint.velocity = start.velocity;
            }
        }
    }
    
    // Remove passed points
    if (closestIndex > 0) {
        path.waypoints.erase(path.waypoints.begin(), path.waypoints.begin() + closestIndex);
    }
    
    // Return lookahead or closest point
    return (tempSmallestLookahead < INFINITY) ? lookaheadPoint : closestPoint;
}

void PurePursuit::calculateDriveSpeeds(const Waypoint& lookahead, int direction,
                                       double& leftVel, double& rightVel) {
    RobotPosition pos = Odometry::getPosition();
    
    double dx = lookahead.x - pos.x;
    double dy = lookahead.y - pos.y;
    
    // Calculate angle to target
    double pointAngle = std::atan2(dy, dx);
    
    // Adjust current angle based on direction
    double adjustedAngle = pos.heading;
    if (direction == -1) {
        adjustedAngle += M_PI;
    }
    
    // Normalize
    adjustedAngle = Utils::normalizeAngle(adjustedAngle);
    
    // Calculate angle difference
    double angleDiff = Utils::normalizeAngle(pointAngle - adjustedAngle);
    
    // Calculate velocities
    double forwardVel = s_forwardVelocity * direction;
    double turnVelK = s_turnVelocityK;
    
    leftVel = forwardVel - angleDiff * turnVelK;
    rightVel = forwardVel + angleDiff * turnVelK;
    
    // Clamp velocities
    leftVel = Utils::clamp(leftVel, -100.0, 100.0);
    rightVel = Utils::clamp(rightVel, -100.0, 100.0);
}

double PurePursuit::calculateCurvature(const RobotPosition& pos, const Waypoint& target) {
    double dx = target.x - pos.x;
    double dy = target.y - pos.y;
    
    double angleToTarget = std::atan2(dy, dx);
    double angleError = Utils::normalizeAngle(angleToTarget - pos.heading);
    
    double distanceToTarget = std::sqrt(dx * dx + dy * dy);
    
    if (distanceToTarget < 0.01) return 0.0;
    
    return (2.0 * std::sin(angleError)) / distanceToTarget;
}

//==========================================================================
// PATH LOADING
//==========================================================================

Path PurePursuit::loadPathFromCSV(const std::string& filename) {
    Path path;
    
    std::string content = Utils::readFile(filename.c_str());
    if (content.empty()) {
        printf("Error: Could not load path from %s\n", filename.c_str());
        return path;
    }
    
    std::vector<std::string> lines = Utils::split(content, '\n');
    
    // Skip header line
    for (size_t i = 1; i < lines.size(); i++) {
        std::string line = Utils::trim(lines[i]);
        if (line.empty()) continue;
        
        std::vector<std::string> values = Utils::split(line, ',');
        if (values.size() < 2) continue;
        
        try {
            double x = std::stod(values[0]);
            double y = std::stod(values[1]);
            double velocity = (values.size() > 3) ? std::stod(values[3]) : 1.0;
            
            path.addWaypoint(x, y, velocity);
        } catch (...) {
            continue;
        }
    }
    
    // Set default parameters
    path.lookaheadDistance = Config::lookahead_distance;
    path.tolerance = Config::DEFAULT_PATH_TOLERANCE;
    
    printf("Loaded path with %d waypoints\n", (int)path.size());
    return path;
}

Path PurePursuit::createPath(const std::vector<std::pair<double, double>>& points) {
    Path path;
    
    for (const auto& point : points) {
        path.addWaypoint(point.first, point.second, 1.0);
    }
    
    path.lookaheadDistance = Config::lookahead_distance;
    path.tolerance = Config::DEFAULT_PATH_TOLERANCE;
    
    return path;
}

Path PurePursuit::mirrorPath(const Path& path, bool mirrorX, bool mirrorY) {
    Path mirrored;
    mirrored.lookaheadDistance = path.lookaheadDistance;
    mirrored.tolerance = path.tolerance;
    mirrored.reverse = path.reverse;
    mirrored.decelerate = path.decelerate;
    mirrored.decelerateDistance = path.decelerateDistance;
    mirrored.decelerateRate = path.decelerateRate;
    mirrored.lastPointTolerance = path.lastPointTolerance;
    
    for (const Waypoint& wp : path.waypoints) {
        double x = mirrorX ? -wp.x : wp.x;
        double y = mirrorY ? -wp.y : wp.y;
        mirrored.addWaypoint(x, y, wp.velocity);
    }
    
    return mirrored;
}

void PurePursuit::stop() {
    s_following = false;
    Hardware::leftDrive.move(0);
    Hardware::rightDrive.move(0);
}

bool PurePursuit::isFollowing() {
    return s_following;
}

void PurePursuit::setForwardVelocity(double velocity) {
    s_forwardVelocity = velocity;
}

void PurePursuit::setTurnVelocityK(double k) {
    s_turnVelocityK = k;
}

double PurePursuit::getForwardVelocity() {
    return s_forwardVelocity;
}

double PurePursuit::getTurnVelocityK() {
    return s_turnVelocityK;
}

