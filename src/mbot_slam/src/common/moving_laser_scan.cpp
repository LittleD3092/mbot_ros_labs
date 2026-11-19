#include "common/moving_laser_scan.hpp"
#include "common/utils.hpp"
#include <cmath>

// Linear interpolation for geometry_msgs::msg::Point
inline geometry_msgs::msg::Point linearInterpolatePoint(const geometry_msgs::msg::Point& a,
                                           const geometry_msgs::msg::Point& b,
                                           float t)
{
    geometry_msgs::msg::Point out;
    out.x = a.x + t * (b.x - a.x);
    out.y = a.y + t * (b.y - a.y);
    out.z = a.z + t * (b.z - a.z);
    return out;
}


MovingLaserScan::MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                                 const Pose& start_pose,
                                 const Pose& end_pose)
{
    deskewScan(scan, start_pose, end_pose);
}

void MovingLaserScan::deskewScan(const sensor_msgs::msg::LaserScan& scan,
                                     const Pose& start_pose,
                                     const Pose& end_pose)
{
    const size_t num_rays = scan.ranges.size();
    rays_.clear();
    rays_.reserve(num_rays);

    // Extract yaw from quaternions (world-frame base yaw at scan start/end)
    const double yaw_start = yawFromQuaternion(start_pose.orientation);
    const double yaw_end = yawFromQuaternion(end_pose.orientation);
    
    // TODO: Calculate how much the robot body has rotated in total - delta_yaw
    // (void)yaw_start; (void)yaw_end;
    const double delta_yaw = std::atan2(
        std::sin(yaw_end - yaw_start),
        std::cos(yaw_end - yaw_start)
    );
    
    // Iterate over all beams
    for (size_t i = 0; i < num_rays; ++i)
    {
        const float range = scan.ranges[i];

        // Drop garbage / under-range / out-of-range
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }

        // TODO: Calculate how much this ray has rotated - delta_lidar
        const float t = (num_rays > 1) ? static_cast<float>(i) / static_cast<float>(num_rays - 1) : 0.0f;
        // Laser angle for this individual ray
        const float delta_lidar = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
        // Hint: To calculate delta_lidar, check LiDAR Scan’s ROS message type “sensor_msgs/LaserScan” 

        // TODO: ray.theta = delta_lidar + a fraction of delta_yaw

        InterpolatedRay ray;
        ray.origin = linearInterpolatePoint(start_pose.position, end_pose.position, t);// Hint: use linearInterpolatePoint()
        ray.theta = static_cast<float>(yaw_start + t * delta_yaw + delta_lidar);
        ray.range = range;  // Does range change?

        rays_.push_back(ray);
    }
}