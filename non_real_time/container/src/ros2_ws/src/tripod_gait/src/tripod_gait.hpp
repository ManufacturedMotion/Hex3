#pragma once

#include <array>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "hexapod_msgs/msg/foot_target.hpp"
#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose.hpp"

class TripodGaitNode : public rclcpp::Node
{
public:
    TripodGaitNode();

private:
    void timerCallback();
    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg);

    void updateGait(double dt);

    bool legInGroupA(size_t leg) const;

    geometry_msgs::msg::Twist cmd_vel_;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::FootTargetArray>::SharedPtr
        foot_target_pub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::BodyPose>::SharedPtr
        body_pose_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_update_;

    double gait_phase_;

    double cycle_time_;
    double step_height_;
    double max_step_length_;
    double body_height_;

    std::array<double, 6> home_x_;
    std::array<double, 6> home_y_;
};