#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <random>
#include <cmath>

class ActionModel
{
public:
    ActionModel();

    // Set the initial odometry baseline so the action model can compute deltas.
    void initialize(const nav_msgs::msg::Odometry& odom);

    // Process a new odometry measurement.
    bool processOdometry(const nav_msgs::msg::Odometry& odom);

    // Propagate a single particle by applying noisy motion from the motion model.
    geometry_msgs::msg::Pose propagateParticle(const geometry_msgs::msg::Pose& pose);

    // Propagate all particles in a cloud and return the resulting proposal cloud.
    nav2_msgs::msg::ParticleCloud propagateParticles(const nav2_msgs::msg::ParticleCloud& cloud);

private:
    nav_msgs::msg::Odometry prev_odom_;

    double k1_{0.005};          // rotational noise coefficient (was 0.015)
    double k2_{0.002};          // translational noise coefficient (was 0.008)
    double min_trans_{0.005}; // [m] wait until at least this much motion has accumulated
    double min_rot_ {0.01};    // [rad] wait until at least this much rotation has accumulated

    // Below this translation [m] the step is treated as a pure rotation (rot1 = 0).
    // Odometry reports a few mm of translation in a random direction while turning in place;
    // without this, rot1 = atan2(dy,dx) can be anything up to +-pi and injects large heading noise.
    static constexpr double kPureRotationTransThresh = 0.005;
    // If the motion direction is more than this far from the heading [rad], the robot drove backwards.
    static constexpr double kReverseAngleThresh = M_PI / 2.0;
    // std::normal_distribution requires stddev > 0.
    static constexpr double kMinStd = 1e-6;

    // (rot1, trans, rot2) motion decomposition
    double rot1_{0.0}, trans_{0.0}, rot2_{0.0};
    /* Corresponding 1-σ values */
    double rot1_std_{0.0}, trans_std_{0.0}, rot2_std_{0.0};

    /* Random-number generator */
    std::mt19937 random_gen;
};