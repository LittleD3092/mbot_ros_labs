#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include "common/moving_laser_scan.hpp"

#include <cmath>

class SensorModel
{
public:
    static constexpr int RAY_STRIDE = 6;
    static constexpr double ADJACENT_FRACTION = 0.5;
    // Beams longer than this [m] are not scored (same reasoning as OccupancyGrid::kMaxMappingRange:
    // they mostly leak through gaps, have the largest angular error, and end in unmapped space).
    static constexpr double MAX_SCORING_RANGE = 5.0;
    // Weight sharpening: weight = exp((score - best_score) / (WEIGHT_TEMPERATURE * best_score)).
    // A particle scoring 10% below the best gets ~e^-1 of its weight at 0.1.
    // Smaller = pickier (tighter cloud, but risk of collapsing); larger = closer to the old linear weights.
    static constexpr double WEIGHT_TEMPERATURE = 0.1;

    explicit SensorModel();

    double likelihood(const MovingLaserScan& scan,
                      const nav_msgs::msg::OccupancyGrid& map) const;

    // Apply sensor model to a proposal cloud: weight each particle by scan likelihood.
    // parent_cloud is the cloud before propagation, used for scan interpolation.
    nav2_msgs::msg::ParticleCloud weightParticles(const nav2_msgs::msg::ParticleCloud& proposal,
                                                  const nav2_msgs::msg::ParticleCloud& parent_cloud,
                                                  const sensor_msgs::msg::LaserScan&   scan,
                                                  const nav_msgs::msg::OccupancyGrid&  map) const;

private:
    // occupancy is probability percentage [0-100]
    double occupancyToLogOdds(int8_t occupancy) const;

    bool worldToGrid(double world_x, double world_y,
                     const nav_msgs::msg::OccupancyGrid& map,
                     int& grid_x, int& grid_y) const;

    bool isCellInGrid(int x, int y, const nav_msgs::msg::OccupancyGrid& map) const;
    int8_t getOccupancy(int x, int y, const nav_msgs::msg::OccupancyGrid& map) const;

    int ray_stride_;
};
