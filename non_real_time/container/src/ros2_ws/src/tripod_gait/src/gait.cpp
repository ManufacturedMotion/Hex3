#include "gait.hpp"

Gait::Gait(
    const std::string& node_name)
    : Node(node_name)
{
    cmd_vel_sub_ =
        create_subscription<
            geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &Gait::cmdVelCallback,
                this,
                std::placeholders::_1));

    foot_array_pub_ =
        create_publisher<
            hexapod_msgs::msg::FootTargetArray>(
            "/foot_targets/array",
            10);

    body_array_pub_ =
        create_publisher<
            hexapod_msgs::msg::BodyPoseArray>(
            "/body_pose/array",
            10);

    body_pose_pub_ =
        create_publisher<
            hexapod_msgs::msg::BodyPose>(
            "/body_pose",
            10);

    foot_target_pub_ =
        create_publisher<
            hexapod_msgs::msg::FootTarget>(
            "/foot_target",
            10);

    timer_ =
        create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(
                &Gait::update,
                this));

    last_update_ = now();
}

void Gait::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    v_command = Pose6D(msg);
}

void Gait::update()
{
    auto current_time = now();

    double dt =
        (current_time - last_update_)
            .seconds();

    last_update_ = current_time;

    updateGait(dt, rclcpp::Duration(current_time));
}

void Gait::publishFootTargetArray(
    const hexapod_msgs::msg::FootTargetArray& msg)
{
    foot_target_array_pub_->publish(msg);
}

void Gait::publishBodyPoseArray(
    const hexapod_msgs::msg::BodyPoseArray& msg)
{
    body_array_pub_->publish(msg);
}

void Gait::publishFootTarget(
    const hexapod_msgs::msg::FootTarget& msg)
{
    foot_target_pub_->publish(msg);
}

void Gait::publishBodyPose(
    const hexapod_msgs::msg::BodyPose& msg)
{
    body_pose_pub_->publish(msg);
}