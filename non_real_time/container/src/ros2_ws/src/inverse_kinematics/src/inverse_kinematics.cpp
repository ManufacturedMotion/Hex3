#include "inverse_kinematics.hpp"

InverseKinematicsNode::InverseKinematicsNode()
    : Node("inverse_kinematics")
{
    foot_target_sub_ =
        create_subscription<
            hexapod_msgs::msg::FootTargetArray>(
            "/foot_targets",
            10,
            std::bind(
                &InverseKinematicsNode::footTargetCallback,
                this,
                std::placeholders::_1));

    body_pose_sub_ =
        create_subscription<
            hexapod_msgs::msg::BodyPose>(
            "/body_pose",
            10,
            std::bind(
                &InverseKinematicsNode::bodyPoseCallback,
                this,
                std::placeholders::_1));

    leg_command_pub_ =
        create_publisher<
            hexapod_msgs::msg::LegCommand>(
            "/leg_commands",
            100);
}

uint8_t InverseKinematicsNode::_inverseKinematics(
    Position pos,
    _Bool active_legs[NUM_LEGS],
    std::array<double, 3> * results
) {
    if (!_preCheckSafePos(pos))
        return 254; // Pre-check fail

    // Divide rotations by 100 to get radians
    pos.roll  *= 0.01;
    pos.pitch *= 0.01;
    pos.yaw   *= 0.01;

    double potential_results[NUM_LEGS][3];

    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        potential_results[i][0] = pos.x;
        potential_results[i][1] = pos.y;
        potential_results[i][2] =
            pos.z
            + sin(pos.pitch) * (_leg_X_offset[i] + pos.x)
            + sin(pos.roll)  * (_leg_Y_offset[i] + pos.y);
    }

    // Apply rotations using temporary std::array
    for (uint8_t i = 0; i < NUM_LEGS; i++) {

        std::array<double, 3> temp = {
            potential_results[i][0],
            potential_results[i][1],
            potential_results[i][2]
        };

        // NOTE: assuming your old ThreeByOne used values[3]
        auto rotateYaw = [&](double yaw) {
            double c = cos(yaw);
            double s = sin(yaw);

            double x = temp[0];
            double y = temp[1];

            temp[0] = x * c - y * s;
            temp[1] = x * s + y * c;
        };

        rotateYaw(_home_yaws[i]);

        temp[0] += _stance_offset[0];
        temp[1] += _stance_offset[1];
        temp[2] += _stance_offset[2];

        rotateYaw(pos.yaw);

        potential_results[i][0] = temp[0];
        potential_results[i][1] = temp[1];
        potential_results[i][2] = temp[2];
    }

    // post-check
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (!_postCheckSafeCoords(
                potential_results[i][0],
                potential_results[i][1],
                potential_results[i][2]))
        {
            return 255;
        }
    }

    uint8_t results_index = 0;

    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (active_legs[i]) {

            results[results_index] = {
                potential_results[i][0],
                potential_results[i][1],
                potential_results[i][2]
            };

            results_index++;
        }
    }

    return 0;
}

void InverseKinematicsNode::footTargetCallback(
    const hexapod_msgs::msg::FootTargetArray::SharedPtr msg)
{
    latest_feet_ = *msg;
    feet_received_ = true;

    process();
}

void InverseKinematicsNode::bodyPoseCallback(
    const hexapod_msgs::msg::BodyPose::SharedPtr msg)
{
    latest_body_pose_ = *msg;
    pose_received_ = true;
}

void InverseKinematicsNode::bodyPoseCallback(
    const hexapod_msgs::msg::BodyPose::SharedPtr msg)
{
    latest_body_pose_ = *msg;
    pose_received_ = true;
}

void InverseKinematicsNode::process()
{
    if(!feet_received_)
    {
        return;
    }

    Position body;

    body.x = latest_body_pose_.x;
    body.y = latest_body_pose_.y;
    body.z = latest_body_pose_.z;

    body.roll = latest_body_pose_.roll;
    body.pitch = latest_body_pose_.pitch;
    body.yaw = latest_body_pose_.yaw;

    bool active_legs[NUM_LEGS] =
    {
        true,
        true,
        true,
        true,
        true,
        true
    };

    ThreeByOne body_offsets[NUM_LEGS];

    uint8_t result =
        _inverseKinematics(
            body,
            active_legs,
            body_offsets);

    if(result != 0)
    {
        RCLCPP_WARN(
            get_logger(),
            "Body IK failed: %u",
            result);

        return;
    }

    for(uint8_t i = 0; i < NUM_LEGS; i++)
    {
        hexapod_msgs::msg::LegCommand cmd;

        cmd.leg_number = i;

        cmd.command_type =
            hexapod_msgs::msg::LegCommand::LINEAR;

        cmd.x =
            latest_feet_.legs[i].x +
            body_offsets[i].values[0];

        cmd.y =
            latest_feet_.legs[i].y +
            body_offsets[i].values[1];

        cmd.z =
            latest_feet_.legs[i].z +
            body_offsets[i].values[2];

        cmd.speed = 100.0f;

        leg_command_pub_->publish(cmd);
    }
}