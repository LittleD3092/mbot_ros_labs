#include "controllers/rtr_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>  // std::clamp
#include <cmath>

void RTRController::setPlan(const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
                            double current_x, double current_y)
{
    if (waypoints.empty()) return;

    goal_queue_.clear();
    for (const auto& p : waypoints) goal_queue_.push_back(p);

    // Skip waypoints already within reach
    while (!goal_queue_.empty()) {
        const auto& front = goal_queue_.front();
        double dx = front.x - current_x;
        double dy = front.y - current_y;
        if (std::sqrt(dx*dx + dy*dy) < dist_thresh_) {
            goal_queue_.erase(goal_queue_.begin());
        } else {
            break;
        }
    }

    if (!goal_queue_.empty()) {
        goal_x_       = goal_queue_.front().x;
        goal_y_       = goal_queue_.front().y;
        goal_theta_   = goal_queue_.front().theta;
        goal_queue_.erase(goal_queue_.begin());
        goal_received_ = true;
        resetPID();  //clears integral and derivative (last error) values left over from previous plan
    } else {
        goal_received_ = false;
    }
}

geometry_msgs::msg::Twist RTRController::computeVelocityCommands(double x, double y, double theta)
{
    geometry_msgs::msg::Twist cmd;
    if (!goal_received_) return cmd;

    double dx           = goal_x_ - x;
    double dy           = goal_y_ - y;
    double target_angle = std::atan2(dy, dx);
    double dist_error     = std::sqrt(dx*dx + dy*dy);
    double angle_error  = wrapToPi(target_angle - theta);

    if (dist_error > dist_thresh_ && std::fabs(angle_error) > angle_thresh_) {
        // TODO - STATE 1: rotate toward target
        // STATE 1: rotate in place toward the waypoint
        ang_error_sum_ = std::clamp(ang_error_sum_ + angle_error * dt_,
                                    -integral_limit_, integral_limit_);
        double ang_deriv = wrapToPi(angle_error - ang_error_last_) / dt_;
        ang_error_last_ = angle_error;

        cmd.angular.z = Kp_ang * angle_error + Ki_ang * ang_error_sum_ + Kd_ang * ang_deriv;
    }
    else if (dist_error > dist_thresh_) {
        // TODO - STATE 2: drive forward with heading correction
        // STATE 2: drive forward with heading correction

        // Linear PID on distance
        lin_error_sum_ = std::clamp(lin_error_sum_ + dist_error * dt_,
                                    -integral_limit_, integral_limit_);
        double lin_deriv = (dist_error - lin_error_last_) / dt_;
        lin_error_last_ = dist_error;

        double v = Kp_lin * dist_error + Ki_lin * lin_error_sum_ + Kd_lin * lin_deriv;
        cmd.linear.x = std::max(0.0, v);   // never reverse in this state

        // Angular PID to hold heading toward the waypoint
        ang_error_sum_ = std::clamp(ang_error_sum_ + angle_error * dt_,
                                    -integral_limit_, integral_limit_);
        double ang_deriv = wrapToPi(angle_error - ang_error_last_) / dt_;
        ang_error_last_ = angle_error;

        cmd.angular.z = Kp_ang * angle_error + Ki_ang * ang_error_sum_ + Kd_ang * ang_deriv;
    }
    else {
        // TODO - STATE 3: rotate to final orientation
        // STATE 3: position reached, rotate to the final orientation
        angle_error = wrapToPi(goal_theta_ - theta);

        if (std::fabs(angle_error) > angle_thresh_) {
            ang_error_sum_ = std::clamp(ang_error_sum_ + angle_error * dt_,
                                        -integral_limit_, integral_limit_);
            double ang_deriv = wrapToPi(angle_error - ang_error_last_) / dt_;
            ang_error_last_ = angle_error;

            cmd.angular.z = Kp_ang * angle_error + Ki_ang * ang_error_sum_ + Kd_ang * ang_deriv;
        } else {
            // Waypoint complete: stop and move on
            cmd.linear.x  = 0.0;
            cmd.angular.z = 0.0;
            resetPID();
            loadNextGoal();
            return cmd;
        }
    }

    // Safety limits
    cmd.linear.x  = std::clamp(cmd.linear.x,  -0.3, 0.3);
    cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);
    return cmd;
}

void RTRController::resetPID()
{
    lin_error_sum_ = lin_error_last_ = 0.0;
    ang_error_sum_ = ang_error_last_ = 0.0;
}

void RTRController::loadNextGoal()
{
    if (!goal_queue_.empty()) {
        goal_x_     = goal_queue_.front().x;
        goal_y_     = goal_queue_.front().y;
        goal_theta_ = goal_queue_.front().theta;
        goal_queue_.erase(goal_queue_.begin());
        RCLCPP_INFO(rclcpp::get_logger("rtr_controller"), "Moving to next goal at %.2f %.2f", goal_x_, goal_y_);
    } else {
        goal_received_ = false;
    }
}
