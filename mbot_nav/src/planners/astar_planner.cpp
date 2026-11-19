#include "planners/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

// Precondition: x1, y1 represents the start point, and x2, y2 represents the end point.
// Postcondition: Returns the Euclidean distance between the two points.
float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

// Precondition: dist_grid is a valid ObstacleDistanceGrid object, (x0, y0) represents the start point, and (x1, y1) represents the end point.
// Postcondition: Returns true if the line segment from (x0, y0) to (x1, y1) is free of obstacles (within inflation_radius_ m), false otherwise.
bool AStarPlanner::isLineFree(const ObstacleDistanceGrid& dist_grid, float x0, float y0, float x1, float y1)
{
    const float step = dist_grid.getResolution() * 0.5f;
    float dx = x1 - x0;
    float dy = y1 - y0;
    float dist = std::hypot(dx, dy);
    float angle = std::atan2(dy, dx);

    for (float d = 0.0f; d <= dist; d += step) {
        float x = x0 + std::cos(angle) * d;
        float y = y0 + std::sin(angle) * d;
        int grid_x = static_cast<int>(std::floor((x - dist_grid.getOrigin().position.x) / dist_grid.getResolution()));
        int grid_y = static_cast<int>(std::floor((y - dist_grid.getOrigin().position.y) / dist_grid.getResolution()));
        if (
            !dist_grid.isCellInGrid(grid_x, grid_y) ||
            dist_grid.getOccupancy(grid_x, grid_y) == -1 ||
            dist_grid.getDistance(grid_x, grid_y) < inflation_radius_
        ){
            return false;
        }
    }
    return true;
}

// Precondition:
// - came_from is a map that came_from[current] = parent, where current is the index of the current node and parent is the index of the parent node. Each index is calculated as y * width + x.
// - goal_idx is the index of the goal node.
// - width is the width of the grid.
//
// Postcondition: Returns a vector of (x, y) pairs representing the waypoints from the start to the goal, in order.
std::vector<std::pair<int, int>> AStarPlanner::backtraceWaypoints(
    const std::unordered_map<int, int>& came_from, int goal_idx, int width)
{
    std::vector<std::pair<int, int>> path;
    int current = goal_idx;

    while (true) {
        path.emplace_back(current % width, current / width);
        int next = came_from.at(current);
        if (next == current) break;  // We found the start node!
        current = next;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

// Precondition:
// - dist_grid is a valid ObstacleDistanceGrid object, which contains the occupancy and distance to obstacles for each cell in the grid.
// - start is the starting pose in world coordinates (x, y, theta).
// - goal is the goal pose in world coordinates (x, y, theta).
// - path is an output parameter that will contain the planned path as a sequence of Pose2D messages in world coordinates.
//
// Postcondition: Returns true if a path was successfully planned from start to goal, false otherwise. If successful, the path parameter will contain the planned path.
bool AStarPlanner::planPath(const ObstacleDistanceGrid& dist_grid,
                            const geometry_msgs::msg::Pose2D& start,
                            const geometry_msgs::msg::Pose2D& goal,
                            mbot_interfaces::msg::Pose2DArray& path) {
    path.poses.clear();

    mbot_interfaces::msg::Pose2DArray raw_path;

    const float res = dist_grid.getResolution();
    const float origin_x = dist_grid.getOrigin().position.x;
    const float origin_y = dist_grid.getOrigin().position.y;
    const int width = dist_grid.getWidth();

    // Start and goal in grid coordinates
    int start_x = static_cast<int>(std::floor((start.x - origin_x) / res));
    int start_y = static_cast<int>(std::floor((start.y - origin_y) / res));
    int goal_x  = static_cast<int>(std::floor((goal.x  - origin_x) / res));
    int goal_y  = static_cast<int>(std::floor((goal.y  - origin_y) / res));

    // Check if the start position is valid
    if (!dist_grid.isCellInGrid(start_x, start_y))
        return false;

    // Check if the goal position is valid
    if (
        !dist_grid.isCellInGrid(goal_x, goal_y) ||
        dist_grid.getOccupancy(goal_x, goal_y) == -1 ||
        dist_grid.getOccupancy(goal_x, goal_y) >= 50 ||
        dist_grid.getDistance(goal_x, goal_y) < inflation_radius_
    )
        return false;

    // Min-priority queue
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<>> open;
    // Hash tables, similar to python's dict
    std::unordered_map<int, float> cost_so_far;
    std::unordered_map<int, int> came_from;

    int start_idx = toIndex(start_x, start_y, width);
    int goal_idx = toIndex(goal_x, goal_y, width);

    open.push({start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y)});
    cost_so_far[start_idx] = 0.0f;
    came_from[start_idx] = start_idx;

    struct Offset { int x; int y; };
    const std::vector<Offset> connected_neighbors = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},   // Cardinal directions
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}  // Diagonals
    };

    // Returns true if the cell can be traversed by the robot
    auto isTraversable = [&](int x, int y) {
        if (!dist_grid.isCellInGrid(x, y)) return false;
        int8_t occupancy = dist_grid.getOccupancy(x, y);
        if (occupancy < 0 || occupancy >= 50) return false;  // unknown or occupied
        return dist_grid.getDistance(x, y) >= inflation_radius_;  // too close to obstacles
    };

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        int curr_idx = toIndex(current.x, current.y, width);

        // Skip stale queue entries
        if (current.cost > cost_so_far[curr_idx]) continue;

        if (curr_idx == goal_idx) {
            // Goal reached: build the world-frame path at cell centers
            auto waypoints_idx = backtraceWaypoints(came_from, goal_idx, width);
            for (size_t i = 0; i < waypoints_idx.size(); ++i) {
                const auto& [x, y] = waypoints_idx[i];
                geometry_msgs::msg::Pose2D pose;
                pose.x = (x + 0.5f) * res + origin_x;
                pose.y = (y + 0.5f) * res + origin_y;
                pose.theta = 0.0f;
                raw_path.poses.push_back(pose);
            }
            raw_path.poses.back().theta = goal.theta;

            // Prune unnecessary waypoints; smoothPath also sets the headings
            path = smoothPath(dist_grid, raw_path);
            return true;
        }

        for (const auto& offset : connected_neighbors) {
            int nx = current.x + offset.x;
            int ny = current.y + offset.y;

            // Step 1: Skip it if it is not valid
            if (!isTraversable(nx, ny)) continue;

            // Prevent diagonal moves from cutting through corners
            if (offset.x != 0 && offset.y != 0) {
                if (!isTraversable(current.x + offset.x, current.y) ||
                    !isTraversable(current.x, current.y + offset.y)) {
                    continue;
                }
            }

            // Step 2: Calculate the cost
            float movement_cost = (offset.x != 0 && offset.y != 0) ? std::sqrt(2.0f) : 1.0f;

            // Penalize cells that are close to obstacles
            float obstacle_dist = dist_grid.getDistance(nx, ny);
            float obstacle_cost = 0.0f;
            const float safety_distance = 0.5f;
            if (obstacle_dist < safety_distance) {
                obstacle_cost = 2.0f * (safety_distance - obstacle_dist) / res;
            }

            float new_cost = current.cost + movement_cost + obstacle_cost;

            // Step 3: Update the cost_so_far, came_from, and open if this path is better
            int neighbor_idx = toIndex(nx, ny, width);
            auto it = cost_so_far.find(neighbor_idx);
            if (it == cost_so_far.end() || new_cost < it->second) {
                cost_so_far[neighbor_idx] = new_cost;
                came_from[neighbor_idx] = curr_idx;
                open.push({nx, ny, new_cost, new_cost + heuristic(nx, ny, goal_x, goal_y)});
            }
        }
    }

    return false;
}

// Precondition:
// - dist_grid is a valid ObstacleDistanceGrid object, which contains the occupancy and distance to obstacles for each cell in the grid.
// - raw_path is a sequence of Pose2D messages in world coordinates that represents the initial path generated by the A* algorithm.
//
// Postcondition: Returns a Pose2DArray containing the smoothed path, where unnecessary waypoints have been pruned while ensuring that the path remains free of obstacles.
// Each waypoint's heading points from the previous waypoint to it; the last waypoint keeps the heading of the last raw waypoint (the goal heading).
mbot_interfaces::msg::Pose2DArray AStarPlanner::smoothPath(
    const ObstacleDistanceGrid& dist_grid, const mbot_interfaces::msg::Pose2DArray& raw_path)
{
    mbot_interfaces::msg::Pose2DArray pruned;
    if (raw_path.poses.empty()) return pruned;

    // Nothing to prune with two or fewer waypoints
    if (raw_path.poses.size() <= 2) {
        return raw_path;
    }

    size_t current = 0;
    pruned.poses.push_back(raw_path.poses.front());

    while (current < raw_path.poses.size() - 1) {
        // Find the farthest waypoint reachable from `current` along a collision-free straight line
        size_t next = raw_path.poses.size() - 1;
        while (next > current + 1) {
            const auto& p1 = raw_path.poses[current];
            const auto& p2 = raw_path.poses[next];
            if (isLineFree(dist_grid, p1.x, p1.y, p2.x, p2.y)) {
                break;
            }
            --next;
        }

        geometry_msgs::msg::Pose2D pose = raw_path.poses[next];
        const auto& previous = raw_path.poses[current];
        pose.theta = std::atan2(pose.y - previous.y, pose.x - previous.x);
        pruned.poses.push_back(pose);
        current = next;
    }

    pruned.poses.back().theta = raw_path.poses.back().theta;
    return pruned;
}
