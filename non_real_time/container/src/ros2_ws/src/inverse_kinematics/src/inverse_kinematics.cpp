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

uint8_t InverseKinematicsNode::_inverseKinematics(Position pos, _Bool active_legs[NUM_LEGS], ThreeByOne * results) {

	if (!_preCheckSafePos(pos))
        return 254; // Pre-check fail
	
	// Divide rotations by 100 to get radians (stored as hundredths of a radian to put on similar scale as x, y, z)
	pos.roll *= 0.01;
	pos.pitch *= 0.01;
	pos.yaw *= 0.01;

	double potential_results[NUM_LEGS][3];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		potential_results[i][0] = pos.x;
		potential_results[i][1] = pos.y;
		potential_results[i][2] = pos.z + sin(pos.pitch) * (_leg_X_offset[i] + pos.x) + sin(pos.roll) * (_leg_Y_offset[i] + pos.y);
	}

	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		ThreeByOne temp = ThreeByOne(potential_results[i][0], potential_results[i][1], potential_results[i][2]);
		temp.rotateYaw(_home_yaws[i]);
		temp += _stance_offset;
		temp.rotateYaw(pos.yaw);
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			potential_results[i][j] = temp.values[j];
		}
	}

	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (!_postCheckSafeCoords(potential_results[i][0], potential_results[i][1], potential_results[i][2]))
			return 255; // Post-check fail
	}
	uint8_t results_index = 0;
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if(active_legs[i]) {
			results[results_index] = ThreeByOne(potential_results[i]);
			results_index++;
		}
		#if LOG_LEVEL >= CALCULATION_LOGGING
			Serial.println("IK value; leg: " + String(i) + " x:" + String(results[i].values[0]) + " y:" + String(results[i].values[1]) + " z:" + String(results[i].values[2]) + "\n");
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