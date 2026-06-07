#pragma once

#include <array>

#include "rclcpp/rclcpp.hpp"

#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose.hpp"
#include "hexapod_msgs/msg/leg_command.hpp"

#define NUM_LEGS 6

struct IKPose {
    double x;
    double y;
    double z;

    double sin_roll;
    double sin_pitch;

    double cos_yaw;
    double sin_yaw;
};


class InverseKinematicsNode : public rclcpp::Node
{
public:
    InverseKinematicsNode();

private:
    void footTargetCallback(
        const hexapod_msgs::msg::FootTargetArray::SharedPtr msg);

    void bodyPoseCallback(
        const hexapod_msgs::msg::BodyPose::SharedPtr msg);

    void process();

    uint8_t _inverseKinematics(
        const IKPose& pose,
        std::array<double, 3> * results);

    rclcpp::Subscription<
        hexapod_msgs::msg::FootTargetArray>::SharedPtr
        foot_target_sub_;

    rclcpp::Subscription<
        hexapod_msgs::msg::BodyPose>::SharedPtr
        body_pose_sub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::LegCommand>::SharedPtr
        leg_command_pub_;

    hexapod_msgs::msg::FootTargetArray latest_feet_;
    hexapod_msgs::msg::BodyPose latest_body_pose_;

    bool feet_received_{false};
    bool pose_received_{false};
};