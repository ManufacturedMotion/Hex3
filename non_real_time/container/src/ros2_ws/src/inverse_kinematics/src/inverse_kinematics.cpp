#include "inverse_kinematics.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

InverseKinematicsNode::InverseKinematicsNode()
: Node("inverse_kinematics")
{
    loadConfig(this->declare_parameter<std::string>("ik_config", "config/ik_config.json"));
    foot_target_sub_ =
        create_subscription<hexapod_msgs::msg::FootTargetArray>(
            "/foot_targets",
            10,
            std::bind(
                &InverseKinematicsNode::footTargetCallback,
                this,
                std::placeholders::_1));

    body_pose_sub_ =
        create_subscription<hexapod_msgs::msg::BodyPose>(
            "/body_pose",
            10,
            std::bind(
                &InverseKinematicsNode::bodyPoseCallback,
                this,
                std::placeholders::_1));

    leg_command_pub_ =
        create_publisher<hexapod_msgs::msg::LegCommand>(
            "/leg_commands",
            100);
    
    timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&InverseKinematicsNode::process, this));
}

void InverseKinematicsNode::_inverseKinematics(
    const IKPose& pose,
    std::array<double, 3>* results
) {


    // --- SoA buffers (SIMD-friendly) ---
    alignas(32) double px[NUM_LEGS];
    alignas(32) double py[NUM_LEGS];
    alignas(32) double pz[NUM_LEGS];

    // --- base stage ---
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        px[i] = pose.x;
        py[i] = pose.y;

        pz[i] =
            pose.z
            + pose.sin_pitch * (_leg_coordinate_transforms[i].x + pose.x)
            + pose.sin_roll  * (_leg_coordinate_transforms[i].y + pose.y);
    }

    // --- transform stage ---
    for (uint8_t i = 0; i < NUM_LEGS; i++) {

        double cy = cos(_leg_coordinate_transforms[i].yaw);
        double sy = sin(_leg_coordinate_transforms[i].yaw);

        double x = px[i];
        double y = py[i];

        // home yaw
        double hx = x * cy - y * sy;
        double hy = x * sy + y * cy;

        // stance offset
        hx += _stance_offset[0];
        hy += _stance_offset[1];
        pz[i] += _stance_offset[2];

        // global yaw (precomputed)
        double nx = hx * pose.cos_yaw - hy * pose.sin_yaw;
        double ny = hx * pose.sin_yaw + hy * pose.cos_yaw;

        px[i] = nx;
        py[i] = ny;
    }

    // --- pack output ---
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        results[i] = { px[i], py[i], pz[i] };
    }

}

void InverseKinematicsNode::footTargetCallback(
    const hexapod_msgs::msg::FootTargetArray::SharedPtr msg)
{
    latest_feet_ = *msg;
    feet_received_ = true;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,   // 1 Hz log rate
        "Received FootTargets: leg0=(%.1f, %.1f, %.1f)",
        msg->foot_targets[0].x, msg->foot_targets[1].y, msg->foot_targets[2].z
    );
}

void InverseKinematicsNode::bodyPoseCallback(
    const hexapod_msgs::msg::BodyPose::SharedPtr msg)
{
    latest_body_pose_ = *msg;
    pose_received_ = true;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,   // 1 Hz log rate
        "Received BodyPose: x=%.2f y=%.2f z=%.2f roll=%.3f pitch=%.3f yaw=%.3f",
        msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw
    );
}

void InverseKinematicsNode::loadConfig(
    const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error(
            "Failed to open IK configuration file: " +
            filename);
    }

    json config;
    file >> config;

    //
    // Stance offset
    //
    const auto& stance =
        config.at("stance_offset_xyz");

    if (stance.size() != 3)
    {
        throw std::runtime_error(
            "stance_offset_xyz must contain exactly 3 values");
    }

    _stance_offset = { stance[0].get<double>(), stance[1].get<double>(), stance[2].get<double>() };
    RCLCPP_INFO(get_logger(),
        "Loaded stance offset: x=%.1f y=%.1f z=%.1f",
        _stance_offset[0], _stance_offset[1], _stance_offset[2]);

    //
    // Leg coordinate transforms
    //
    const auto& transforms =
        config.at("leg_coordinate_transforms");

    if (transforms.size() != NUM_LEGS)
    {
        throw std::runtime_error(
            "leg_coordinate_transforms size does not match NUM_LEGS");
    }

    for (const auto& leg : transforms)
    {
        const uint8_t id =
            leg.at("id").get<uint8_t>();

        if (id >= NUM_LEGS)
        {
            throw std::runtime_error(
                "Invalid leg id in configuration");
        }

        _leg_coordinate_transforms[id].x =
            leg.at("x").get<double>();

        _leg_coordinate_transforms[id].y =
            leg.at("y").get<double>();

        _leg_coordinate_transforms[id].z =
            leg.at("z").get<double>();

        _leg_coordinate_transforms[id].roll =
            leg.at("roll").get<double>();

        _leg_coordinate_transforms[id].pitch =
            leg.at("pitch").get<double>();

        _leg_coordinate_transforms[id].yaw =
            leg.at("yaw").get<double>();
    }
}

void InverseKinematicsNode::process()
{
    if (!feet_received_ || !pose_received_) {
        return;
    }

    // --- Build SIMD-friendly pose ---
    IKPose pose;

    pose.x = latest_body_pose_.x;
    pose.y = latest_body_pose_.y;
    pose.z = latest_body_pose_.z;

    double roll  = latest_body_pose_.roll * 0.01;
    double pitch = latest_body_pose_.pitch * 0.01;
    double yaw   = latest_body_pose_.yaw * 0.01;

    pose.sin_roll  = sin(roll);
    pose.sin_pitch = sin(pitch);
    pose.sin_yaw   = sin(yaw);
    pose.cos_yaw   = cos(yaw);

    std::array<double, 3> body_offsets[NUM_LEGS];

    _inverseKinematics(
        pose,
        body_offsets);

    for (uint8_t i = 0; i < NUM_LEGS; i++) {

        hexapod_msgs::msg::LegCommand cmd;

        cmd.leg_number = i;
        cmd.command_type = hexapod_msgs::msg::LegCommand::RAPID;

        cmd.x = latest_feet_.foot_targets[i].x + body_offsets[i][0];
        cmd.y = latest_feet_.foot_targets[i].y + body_offsets[i][1];
        cmd.z = latest_feet_.foot_targets[i].z + body_offsets[i][2];

        // cmd.speed = 200.0f;

        leg_command_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,   // 1 Hz log rate
            "Published LegCommand for leg %d: x=%.1f y=%.1f z=%.1f",
            i, cmd.x, cmd.y, cmd.z
        );
    }
}