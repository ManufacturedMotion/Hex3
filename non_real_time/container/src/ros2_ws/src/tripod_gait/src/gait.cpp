#include "gait.hpp"
#include "pose.hpp"

Gait::Gait(
    const std::string& node_name,
    Pose6D multiplier)
    : Node(node_name)
{   
    v_command_multiplier = multiplier;
    cmd_vel_sub_ =
        create_subscription<
            geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &Gait::cmdVelCallback,
                this,
                std::placeholders::_1));

    cmd_vel_sub_ =
    create_subscription<
        std_msgs::msg::Int8>(
        "/macro",
        10,
        std::bind(
            &Gait::cmdMacroCallback,
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

void Gait::cmdMacroCallback(
    const std_msgs::msg::Int8::SharedPtr msg)
{
    runMacro(msg->data);
}


void Gait::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    v_command = Pose6D(msg);
    v_command *= v_command_multiplier;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
    "received cmd_vel x:%f; y:%f; z:%f, roll:%f, pitch:%f, yaw:%f", 
    v_command.x, 
    v_command.y, 
    v_command.z, 
    v_command.roll, 
    v_command.pitch, 
    v_command.yaw);
}

void Gait::updateGait(
    double dt,
    rclcpp::Duration current_time)
{
}

void Gait::runMacro(int8_t macro_num)
{
}

void Gait::update()
{
    auto current_time = now();

    double dt =
        (current_time - last_update_)
            .seconds();

    last_update_ = current_time;
    updateGait(dt, rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(current_time.nanoseconds())));
}

void Gait::publishFootTargetArray(
    const hexapod_msgs::msg::FootTargetArray& msg)
{
    foot_array_pub_->publish(msg);
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