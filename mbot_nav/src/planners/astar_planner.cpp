#include "planners/astar_planner.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

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
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float dist = std::hypot(dx, dy);
    const float angle = std::atan2(dy, dx);

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
        if (next == current) break; // We found the start node!
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
    const float origin_x  = dist_grid.getOrigin().position.x;
    const float origin_y  = dist_grid.getOrigin().position.y;
    const int width = dist_grid.getWidth();

    // start and goal in grid coordinates
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
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},  // Cardinal directions
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonals
    };

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        int curr_idx = toIndex(current.x, current.y, width);
        if (curr_idx == goal_idx) {
            // goal reached, generate the path
            auto waypoints_idx = backtraceWaypoints(came_from, goal_idx, width);

            // Step 1: Calculate raw_path with postions only
            for(size_t i = 0; i < waypoints_idx.size(); ++i)
            {
                const auto& [x, y] = waypoints_idx[i];
                geometry_msgs::msg::Pose2D pose;
                pose.x = (x + 0.5f) * res + origin_x;
                pose.y = (y + 0.5f) * res + origin_y;
                raw_path.poses.push_back(pose);
            }
            
            // Step 2: Smooth the path to remove unnecessary waypoints
            path = smoothPath(dist_grid, raw_path);
            
            // Step 3: Calculate the heading based on the smoothed path
            for(size_t i = 0; i < path.poses.size(); ++i)
            {
                if(i + 1 < path.poses.size())
                {
                    const auto& next_pose = path.poses[i + 1];
                    const auto& current_pose = path.poses[i];
                    path.poses[i].theta = std::atan2(
                        next_pose.y - current_pose.y, 
                        next_pose.x - current_pose.x
                    );
                }
                else // last waypoint, use goal heading
                {
                    path.poses[i].theta = goal.theta;
                }
            }
            return true;
        }

        for (const auto& offset : connected_neighbors) {
            int nx = current.x + offset.x;
            int ny = current.y + offset.y;

            // Step 1: Skip it if it is not valid
            if(
                // not in grid
                !dist_grid.isCellInGrid(nx, ny) ||
                // unknown
                dist_grid.getOccupancy(nx, ny) == -1 ||
                // occupied
                dist_grid.getOccupancy(nx, ny) >= 50 ||
                // too close to obstacles
                dist_grid.getDistance(nx, ny) < inflation_radius_
            )
                continue;

            // Step 2: Calculate the cost
            float new_cost = current.cost + (
                offset.x != 0 && offset.y != 0 ? // diagonal movement?
                std::sqrt(2.0f) : 1.0f
            );

            // Step 3: Update the cost_so_far, came_from, and open if this path is better
            int neighbor_idx = toIndex(nx, ny, width);
            if(
                cost_so_far.find(neighbor_idx) == cost_so_far.end() || 
                new_cost < cost_so_far[neighbor_idx]
            )
            {
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
mbot_interfaces::msg::Pose2DArray AStarPlanner::smoothPath(
    const ObstacleDistanceGrid& dist_grid, const mbot_interfaces::msg::Pose2DArray& raw_path)
{
    mbot_interfaces::msg::Pose2DArray pruned;
    if (raw_path.poses.empty()) return pruned;

    // Prune unnecessary waypoints by checking if the line between two waypoints is free of obstacles
    const size_t n = raw_path.poses.size();
    size_t i = 0;
    pruned.poses.push_back(raw_path.poses[0]);
    while(i < n - 1)
    {
        size_t j = i + 1;
        while(
            j + 1 < n && 
            isLineFree(
                dist_grid, raw_path.poses[i].x, raw_path.poses[i].y, 
                raw_path.poses[j + 1].x, raw_path.poses[j + 1].y
            )
        )
        {
            ++j;
        }
        pruned.poses.push_back(raw_path.poses[j]);
        i = j;
    }
    return pruned;
}
