#include "inverse_kinematics.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

InverseKinematicsNode::InverseKinematicsNode()
: Node("inverse_kinematics")
{
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        pose_received_[i] = false;
        feet_received_[i] = false;
    }
    loadConfig(this->declare_parameter<std::string>("ik_config", "config/ik_config.json"));
    foot_target_array_sub_ =
        create_subscription<hexapod_msgs::msg::FootTargetArray>(
            "/foot_targets",
            10,
            std::bind(
                &InverseKinematicsNode::footTargetArrayCallback,
                this,
                std::placeholders::_1));

    foot_target_sub_ =
        create_subscription<hexapod_msgs::msg::FootTarget>(
            "/foot_target",
            10,
            std::bind(
                &InverseKinematicsNode::footTargetCallback,
                this,
                std::placeholders::_1));

    body_pose_array_sub_ =
        create_subscription<hexapod_msgs::msg::BodyPoseArray>(
            "/body_pose/array",
            10,
            std::bind(
                &InverseKinematicsNode::bodyPoseArrayCallback,
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
        std::chrono::milliseconds(10),
        std::bind(&InverseKinematicsNode::process, this));
}

static inline void rotateVectorRPY(
    std::array<double, 3>& v,
    double roll,
    double pitch,
    double yaw)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);

    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // Save original values before overwriting
    const double x = v[0];
    const double y = v[1];
    const double z = v[2];

    // Rz * Ry * Rx
    v[0] =
        cy * cp * x +
        (cy * sp * sr - sy * cr) * y +
        (cy * sp * cr + sy * sr) * z;

    v[1] =
        sy * cp * x +
        (sy * sp * sr + cy * cr) * y +
        (sy * sp * cr - cy * sr) * z;

    v[2] =
        -sp * x +
        cp * sr * y +
        cp * cr * z;
}

void InverseKinematicsNode::_inverseKinematics(
    const std::array<IKPose, NUM_LEGS>& poses,
    std::array<double, 3>* results
) {
	std::array<double, 3> potential_results[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
        const IKPose& pose = poses[i];
		potential_results[i][0] = pose.x;
		potential_results[i][1] = pose.y;
		potential_results[i][2] = pose.z + sin(pose.pitch) * (_leg_coordinate_transforms[i].x + pose.x) + sin(pose.roll) * (_leg_coordinate_transforms[i].y + pose.y);

		rotateVectorRPY(potential_results[i], _leg_coordinate_transforms[i].roll, _leg_coordinate_transforms[i].pitch, _leg_coordinate_transforms[i].yaw);
		potential_results[i][0] += _stance_offset[0];
		potential_results[i][1] += _stance_offset[1];
		potential_results[i][2] += _stance_offset[2];
        rotateVectorRPY(potential_results[i], 0.0, 0.0, pose.yaw);
	}

	for (uint8_t i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < 3; j++) {
            results[i][j] = potential_results[i][j];
        }
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *get_clock(),
            1000,  // milliseconds
            "Leg %d body offset: x=%.1f y=%.1f z=%.1f",
            i, results[i][0], results[i][1], results[i][2]
        );
    }

}

void InverseKinematicsNode::footTargetCallback(
    const hexapod_msgs::msg::FootTarget::SharedPtr msg)
{   
    feet_received_[msg->leg_number] = true;
    latest_feet_.foot_targets[msg->leg_number] = *msg;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,   // 1 Hz log rate
        "Received FootTarget: x=%.1f y=%.1f z=%.1f",
        msg->x, msg->y, msg->z
    );
}

void InverseKinematicsNode::footTargetArrayCallback(
    const hexapod_msgs::msg::FootTargetArray::SharedPtr msg)
{
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        feet_received_[i] = true;
    }
    latest_feet_ = *msg;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,   // 1 Hz log rate
        "Received FootTargets: leg0=(%.1f, %.1f, %.1f)",
        msg->foot_targets[0].x, msg->foot_targets[1].y, msg->foot_targets[2].z
    );
}

void InverseKinematicsNode::bodyPoseArrayCallback(
    const hexapod_msgs::msg::BodyPoseArray::SharedPtr msg)
{
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        pose_received_[i] = true;
    }
    latest_body_poses_ = *msg;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,   // 1 Hz log rate
        "Received BodyPoseArray: leg0 x=%.2f y=%.2f z=%.2f roll=%.3f pitch=%.3f yaw=%.3f",
        msg->body_poses[0].x, msg->body_poses[0].y, msg->body_poses[0].z,
        msg->body_poses[0].roll, msg->body_poses[0].pitch, msg->body_poses[0].yaw
    );
}

void InverseKinematicsNode::bodyPoseCallback(
    const hexapod_msgs::msg::BodyPose::SharedPtr msg)
{
    pose_received_[msg->leg_number] = true;
    latest_body_poses_.body_poses[msg->leg_number] = *msg;
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
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,  // milliseconds
        "Loaded stance offset: x=%.1f y=%.1f z=%.1f",
        _stance_offset[0],
        _stance_offset[1],
        _stance_offset[2]);

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

    // --- Build per-leg poses ---
    std::array<IKPose, NUM_LEGS> poses;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        const auto& body_pose = latest_body_poses_.body_poses[i];
        poses[i] = {
            body_pose.x,
            body_pose.y,
            body_pose.z,
            body_pose.roll,
            body_pose.pitch,
            body_pose.yaw
        };
    }

    std::array<double, 3> body_offsets[NUM_LEGS];

    _inverseKinematics(
        poses,
        body_offsets);

    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (!feet_received_[i] || !pose_received_[i]) {
            continue;
        }
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
            *get_clock(),
            1000,  // milliseconds
            "Published LegCommand for leg %d: x=%.1f y=%.1f z=%.1f",
            i, cmd.x, cmd.y, cmd.z
        );
    }
}