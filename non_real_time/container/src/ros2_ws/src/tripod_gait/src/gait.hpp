#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose_array.hpp"
#include "pose.hpp"

class Gait : public rclcpp::Node
{
public:
    explicit Gait(
        const std::string& node_name);

protected:
    virtual void updateGait(
        double dt, double current_time) = 0;

    void publishFootTargets(
        const hexapod_msgs::msg::FootTargetArray& msg);

    void publishBodyPoses(
        const hexapod_msgs::msg::BodyPoseArray& msg);

    Pose6D v_command = Pose6D();

private:
    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg);

    void update();

protected:
    rclcpp::Publisher<
        hexapod_msgs::msg::FootTargetArray>::SharedPtr
        foot_pub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::BodyPoseArray>::SharedPtr
        body_pub_;

private:
    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_update_;
};