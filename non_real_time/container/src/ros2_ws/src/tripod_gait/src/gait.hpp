#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"

#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose_array.hpp"
#include "pose.hpp"
#include "constants.hpp"

class Gait : public rclcpp::Node
{
public:
    explicit Gait(
        const std::string& node_name,
        Pose6D multiplier);

protected:
    virtual void updateGait(
        double dt, rclcpp::Duration current_time);

    virtual void runMacro(int8_t macro_num);

    void publishFootTargetArray(
        const hexapod_msgs::msg::FootTargetArray& msg);

    void publishBodyPoseArray(
        const hexapod_msgs::msg::BodyPoseArray& msg);

    void publishFootTarget(
        const hexapod_msgs::msg::FootTarget& msg);

    void publishBodyPose(
        const hexapod_msgs::msg::BodyPose& msg);

    Pose6D v_command = Pose6D();
    Pose6D v_command_multiplier = Pose6D();

private:
    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg);

    void cmdMacroCallback(
        const std_msgs::msg::Int8::SharedPtr msg);

    void update();

protected:
    rclcpp::Publisher<
        hexapod_msgs::msg::FootTargetArray>::SharedPtr
        foot_array_pub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::BodyPoseArray>::SharedPtr
        body_array_pub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::BodyPose>::SharedPtr
        body_pose_pub_; 

    rclcpp::Publisher<
        hexapod_msgs::msg::FootTarget>::SharedPtr
        foot_target_pub_;

private:
    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_sub_;
    
    rclcpp::Subscription<
        std_msgs::msg::Int8>::SharedPtr
        cmd_macro_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_update_ = now();
};