#include "planners/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

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
        int grid_x = static_cast<int>((x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
        int grid_y = static_cast<int>((y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());
        if (!dist_grid.isCellInGrid(grid_x, grid_y) || dist_grid.getDistance(grid_x, grid_y) < 0.15f) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<int, int>> AStarPlanner::backtraceWaypoints(
    const std::unordered_map<int, int>& came_from, int goal_idx, int width)
{
    std::vector<std::pair<int, int>> path;
    int current = goal_idx;

    while (true) {
        path.emplace_back(current % width, current / width);
        int next = came_from.at(current);
        if (next == current) break;  // reached the start node
        current = next;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

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
    int start_x = static_cast<int>((start.x - origin_x) / res);
    int start_y = static_cast<int>((start.y - origin_y) / res);
    int goal_x = static_cast<int>((goal.x - origin_x) / res);
    int goal_y = static_cast<int>((goal.y - origin_y) / res);

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<>> open;
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

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        int curr_idx = toIndex(current.x, current.y, width);
        if (curr_idx == goal_idx) {
            // Goal reached: build the world-frame path
            auto waypoints_idx = backtraceWaypoints(came_from, goal_idx, width);
            for (size_t i = 0; i < waypoints_idx.size(); ++i) {
                const auto& [x, y] = waypoints_idx[i];
                geometry_msgs::msg::Pose2D pose;
                pose.x = x * res + origin_x;
                pose.y = y * res + origin_y;
                if (i + 1 < waypoints_idx.size()) {
                    const auto& next = waypoints_idx[i + 1];
                    pose.theta = std::atan2(next.second - y, next.first - x);
                } else {
                    pose.theta = goal.theta;
                }
                raw_path.poses.push_back(pose);
            }
            path = smoothPath(dist_grid, raw_path);
            return true;
        }

        for (const auto& offset : connected_neighbors) {
            int nx = current.x + offset.x;
            int ny = current.y + offset.y;

            if (!dist_grid.isCellInGrid(nx, ny)) {
                continue;
            }

            // Skip unknown or occupied cells
            int8_t occupancy = dist_grid.getOccupancy(nx, ny);
            if (occupancy < 0 || occupancy >= 50) {
                continue;
            }

            // Skip cells too close to obstacles
            float obstacle_dist = dist_grid.getDistance(nx, ny);
            if (obstacle_dist < 0.15f) {
                continue;
            }

            // Prevent diagonal moves from cutting through corners
            if (offset.x != 0 && offset.y != 0) {
                int8_t occ1 = dist_grid.getOccupancy(current.x + offset.x, current.y);
                int8_t occ2 = dist_grid.getOccupancy(current.x, current.y + offset.y);
                if (occ1 < 0 || occ1 >= 50 ||
                    occ2 < 0 || occ2 >= 50 ||
                    dist_grid.getDistance(current.x + offset.x, current.y) < 0.15f ||
                    dist_grid.getDistance(current.x, current.y + offset.y) < 0.15f) {
                    continue;
                }
            }

            float movement_cost = (offset.x != 0 && offset.y != 0) ? std::sqrt(2.0f) : 1.0f;

            // Penalize cells that are close to obstacles
            float obstacle_cost = 0.0f;
            const float safety_distance = 0.5f;
            if (obstacle_dist < safety_distance) {
                obstacle_cost = 2.0f * (safety_distance - obstacle_dist) / res;
            }

            float new_cost = cost_so_far[curr_idx] + movement_cost + obstacle_cost;
            int next_idx = toIndex(nx, ny, width);
            auto it = cost_so_far.find(next_idx);
            if (it == cost_so_far.end() || new_cost < it->second) {
                cost_so_far[next_idx] = new_cost;
                float priority = new_cost + heuristic(nx, ny, goal_x, goal_y);
                open.push({nx, ny, new_cost, priority});
                came_from[next_idx] = curr_idx;
            }
        }
    }

    return false;
}

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