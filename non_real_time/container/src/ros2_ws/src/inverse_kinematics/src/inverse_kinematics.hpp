#pragma once

#include <array>

#include "rclcpp/rclcpp.hpp"

#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose_array.hpp"
#include "hexapod_msgs/msg/leg_command.hpp"

#define NUM_LEGS 6

struct IKPose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct LegCoordinateTransform {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};


class InverseKinematicsNode : public rclcpp::Node
{
public:
    InverseKinematicsNode();

private:

    std::array<LegCoordinateTransform, NUM_LEGS> _leg_coordinate_transforms;
    std::array<double, 3> _stance_offset;

    rclcpp::TimerBase::SharedPtr timer_;

    void loadConfig(const std::string& config_path);

    void footTargetCallback(
        const hexapod_msgs::msg::FootTarget::SharedPtr msg);

    void bodyPoseCallback(
        const hexapod_msgs::msg::BodyPose::SharedPtr msg);

    void footTargetArrayCallback(
        const hexapod_msgs::msg::FootTargetArray::SharedPtr msg);

    void bodyPoseArrayCallback(
        const hexapod_msgs::msg::BodyPoseArray::SharedPtr msg);

    void process();

    void _inverseKinematics(
        const std::array<IKPose, NUM_LEGS>& poses,
        std::array<double, 3> * results);

    rclcpp::Subscription<
        hexapod_msgs::msg::FootTargetArray>::SharedPtr
        foot_target_array_sub_;

    rclcpp::Subscription<
        hexapod_msgs::msg::BodyPoseArray>::SharedPtr
        body_pose_array_sub_;

    rclcpp::Subscription<
        hexapod_msgs::msg::BodyPose>::SharedPtr
        body_pose_sub_;

    rclcpp::Subscription<
        hexapod_msgs::msg::FootTarget>::SharedPtr
        foot_target_sub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::LegCommand>::SharedPtr
        leg_command_pub_;

    hexapod_msgs::msg::FootTargetArray latest_feet_;
    hexapod_msgs::msg::BodyPoseArray latest_body_poses_;

    std::array<bool, NUM_LEGS> feet_received_;
    std::array<bool, NUM_LEGS> pose_received_;
};