#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <limits>
#include <cstdint>

class OccupancyGrid {
public:
    // Beams longer than this [m] only clear free space up to the cap and do NOT mark an
    // occupied endpoint. Long beams are mostly ones leaking through gaps between wall panels
    // (and have the largest angular error), which leaves stray specks outside the walls.
    static constexpr float kMaxMappingRange = 5.0f;

    OccupancyGrid(float width_m, float height_m, float resolution, float origin_x, float origin_y);

    void markCellOccupied(int x, int y);
    void markCellFree(int x, int y);

    // Convert log-odds to probability percentage [0-100]
    // -1 for unknown cells (ROS format)
    std::vector<int8_t> getOccupancyGrid() const;

    std::pair<int, int> worldToGrid(float x, float y) const;
    int gridToIndex(int x, int y) const;

    float getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getOriginX() const { return origin_x_; }
    float getOriginY() const { return origin_y_; }

private:
    float resolution_;
    float origin_x_, origin_y_;
    int width_, height_;
    std::vector<float> log_odds_;  // flat row-major grid; NaN = unknown

    const float kHitLogOdds = 0.55f;   // < 0.62 (= occupied_thresh 0.65) so one stray hit is NOT a wall; 2 hits are
    const float kMissLogOdds = -0.2f;  // keeps ~3:1 hit:miss ratio
    const float kMinLogOdds = -2.0f;
    const float kMaxLogOdds = 4.0f;    // was 3.5 (confirmed walls harder to flip)
};

// Bresenham-style ray tracing from sensor origin to hit
std::vector<std::pair<int, int>> bresenhamRayTrace(
    float origin_x, float origin_y, float theta, float range, const OccupancyGrid& grid);