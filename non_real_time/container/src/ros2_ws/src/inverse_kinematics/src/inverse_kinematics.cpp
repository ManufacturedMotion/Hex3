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
    const IKPose& pose,
    const bool active_legs[NUM_LEGS],
    std::array<double, 3>* results
) {
    if (!_preCheckSafePos({pose.x, pose.y, pose.z}))
        return 254;

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
            + pose.sin_pitch * (_leg_X_offset[i] + pose.x)
            + pose.sin_roll  * (_leg_Y_offset[i] + pose.y);
    }

    // --- transform stage ---
    for (uint8_t i = 0; i < NUM_LEGS; i++) {

        double cy = cos(_home_yaws[i]);
        double sy = sin(_home_yaws[i]);

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

    // --- validation ---
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (!_postCheckSafeCoords(px[i], py[i], pz[i]))
            return 255;
    }

    // --- pack output ---
    uint8_t k = 0;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (active_legs[i]) {
            results[k++] = { px[i], py[i], pz[i] };
        }

        #if LOG_LEVEL >= CALCULATION_LOGGING
                Serial.println(
                    "IK leg " + String(i) +
                    " x:" + String(px[i]) +
                    " y:" + String(py[i]) +
                    " z:" + String(pz[i])
                );
        #endif
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