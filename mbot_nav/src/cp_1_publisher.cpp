#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"

class CP1Publisher : public rclcpp::Node {
public:
    CP1Publisher() : Node("cp_1_publisher") {
        pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", rclcpp::QoS(1).transient_local());

        mbot_interfaces::msg::Pose2DArray goal_array;
        // Task 1.4: Nine waypoints
        // Final orientations are specified in degrees.
        // The pose() function converts them to radians.

        goal_array.poses = {
            pose(0.61,  0.00, -90.0),  // Waypoint 1
            pose(0.61, -0.61,   0.0),  // Waypoint 2
            pose(1.22, -0.61,  90.0),  // Waypoint 3
            pose(1.22,  0.61,   0.0),  // Waypoint 4
            pose(1.83,  0.61, -90.0),  // Waypoint 5
            pose(1.83, -0.61,   0.0),  // Waypoint 6
            pose(2.44, -0.61,  90.0),  // Waypoint 7
            pose(2.44,  0.00,   0.0),  // Waypoint 8
            pose(3.25,  0.00,   0.0)   // Waypoint 9
        };
        pub_->publish(goal_array);
        RCLCPP_INFO(this->get_logger(), "Task 1.4: Published %zu waypoints successfully.", goal_array.poses.size());
    }

private:
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr pub_;

    static geometry_msgs::msg::Pose2D pose(double x, double y, double theta_deg) {
        geometry_msgs::msg::Pose2D p;
        p.x = x;
        p.y = y;
        p.theta = theta_deg * M_PI / 180.0;
        return p;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CP1Publisher>());
    rclcpp::shutdown();
    return 0;
}