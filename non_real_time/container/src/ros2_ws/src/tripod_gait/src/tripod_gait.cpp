#include "tripod_gait.hpp"
#include "step_queue.hpp"
#include <cmath>

TripodGaitNode::TripodGaitNode()
    : Gait("tripod_gait", Pose6D(
        MAX_SPEED, 
        MAX_SPEED, 
        MAX_SPEED, 
        MAX_SPEED / ROTATION_MAGNITUDE_SCALE, 
        MAX_SPEED / ROTATION_MAGNITUDE_SCALE, 
        MAX_SPEED / ROTATION_MAGNITUDE_SCALE))
{
    RCLCPP_INFO(get_logger(), "TripodGaitNode started");
}

rclcpp::Duration TripodGaitNode::enqueueMaxStepInDirection_(Pose6D direction_vector, double scalar) {
    RCLCPP_INFO(get_logger(),
        "Direction vector is: x: %f, y: %f, x: %f, roll: %f, pitch: %f, yaw: %f\n scalar: %f",
        direction_vector.x,
        direction_vector.y,
        direction_vector.z,
        direction_vector.roll,
        direction_vector.pitch,
        direction_vector.yaw,
        scalar
    );
    Pose6D buffer0;

	direction_vector.z = 0.00; // For now we don't consider Z, roll, or pitch
	direction_vector.roll = 0.00;
	direction_vector.pitch = 0.00;
	if (direction_vector.scaledMagnitude() < 0.001 || std::isnan(direction_vector.x)) {
        // RCLCPP_INFO(get_logger(), "no step to take");
		return rclcpp::Duration::from_nanoseconds(0); // No step to take
	}
	double speed = scalar * max_step_speed_;

	rclcpp::Duration walk_time = rclcpp::Duration::from_nanoseconds(0);
	double max_step_magnitude_with_flip = getMaxStepMagnitudeInDirection_(direction_vector, true);
	double max_step_magnitude_without_flip = getMaxStepMagnitudeInDirection_(direction_vector, false);
	double max_step_magnitude = max_step_magnitude_without_flip;
	if (max_step_magnitude_with_flip > max_step_magnitude_without_flip) {
		next_step_type_ = static_cast<decltype(next_step_type_)>(static_cast<uint8_t>(next_step_type_) ^ 0x01);
		max_step_magnitude = max_step_magnitude_with_flip;
        RCLCPP_INFO(get_logger(), "Flipping step group, next step type %d", static_cast<uint8_t>(next_step_type_));
	}
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "max step magnitude: %f", max_step_magnitude);
	if (max_step_magnitude < getMaxStepMagnitude_() * 0.5) {
		buffer0 = step_queue_.getCurrentQueueEndPos();
		buffer0.x = 0.00;
		buffer0.y = 0.00;
		buffer0.yaw = 0.00;
		walk_time += step_queue_.enqueue(buffer0, speed, StepType::RETURN_TO_NEUTRAL);
        RCLCPP_INFO(get_logger(), "enqueueing return to neutral");
	}

	Pose6D step_vector = direction_vector.unitVector() * max_step_magnitude;
    step_vector.roll /= ROTATION_MAGNITUDE_SCALE;
    step_vector.pitch /= ROTATION_MAGNITUDE_SCALE;
    step_vector.yaw /= ROTATION_MAGNITUDE_SCALE;
    
	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Enqueueing step vector: x: %f, y: %f, z: %f, roll: %f, pitch %f, yaw: %f", 
        step_vector.x,
        step_vector.y,
        step_vector.z,
        step_vector.roll,
        step_vector.pitch,
        step_vector.yaw
    );
    walk_time += step_queue_.enqueue(step_vector * fabs(scalar), speed, next_step_type_);
	return walk_time;
}

void TripodGaitNode::runMacro(int8_t macro_num) {
    switch(static_cast<MacroCode>(macro_num))
    {
        case MacroCode::STAND:
            step_queue_.enqueue(Pose6D(0, 0, -260, 0, 0, 0), 100, StepType::RAPID_MOVE);
        break;
        case MacroCode::SIT:
            step_queue_.enqueue(Pose6D(0, 0, -180, 0, 0, 0), 100, StepType::LINEAR_MOVE_ABSOLUTE);
        break;
        case MacroCode::FLIP:
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LINEAR_MOVE_ABSOLUTE);
            // step_queue_.enqueue(Pose6D(0, 0, -220, 0, 0, M_PI), 100, StepType::LINEAR_MOVE_ABSOLUTE);
            // step_queue_.enqueue(Pose6D(0, 0, -220, 0, 0, M_PI), 100, StepType::RAPID_MOVE);
        case MacroCode::WAVE:
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, 240, 0, 1.5, 0), 100, StepType::LEG_5_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LEG_5_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, 240, 0, 1.5, 0), 100, StepType::LEG_4_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LEG_4_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, 240, 0, 1.5, 0), 100, StepType::LEG_3_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LEG_3_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, 240, 0, 1.5, 0), 100, StepType::LEG_2_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LEG_2_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, 240, 0, 1.5, 0), 100, StepType::LEG_1_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LEG_1_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, 240, 0, 1.5, 0), 100, StepType::LEG_0_LINEAR_MOVE_ABSOLUTE);
            step_queue_.enqueue(Pose6D(0, 0, -240, 0, 1.5, 0), 100, StepType::LEG_0_LINEAR_MOVE_ABSOLUTE);
            
        default:
        break;
    } 
}


void TripodGaitNode::updateGait(
    double dt, rclcpp::Duration current_time)
{
    hexapod_msgs::msg::FootTargetArray foot_targets;
    hexapod_msgs::msg::BodyPoseArray next_body_poses;
    hexapod_msgs::msg::BodyPose base_body_pose = current_pos_.toBodyPose();
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        next_body_poses.body_poses[i] = base_body_pose;
    }

    if (step_in_progress_) {
        double step_progress = (current_time.seconds() - move_start_time_.seconds()) / move_time_.seconds();
        Pose6D next_pos;
        if (step_progress >= 1.0) {
            // Step complete
            step_in_progress_ = false;
        } else {
            switch(current_step_type_) {
                case StepType::GROUP0:
                case StepType::GROUP1:
                    {
                        std::array<bool, NUM_LEGS> active_legs;
                        next_pos = (end_pos_ - start_pos_) * step_progress + start_pos_;
                        uint8_t step_group = static_cast<uint8_t>(current_step_type_);
                        for (uint8_t i = 0; i < NUM_LEGS / NUM_STEP_GROUPS; i++) {
                            active_legs[step_groups_[step_group][i]] = true;
                            active_legs[step_groups_[(step_group^1)][i]] = false;
                        }
                        rapidMove(next_pos, active_legs, true);
                        step_group ^= 1; 
                        for (uint8_t i = 0; i < NUM_LEGS / 2; i++) {
                            active_legs[step_groups_[step_group][i]] = true;
                            active_legs[step_groups_[(step_group^1)][i]] = false;
                        }
                        next_pos.z += -4 * step_progress * (step_progress - 1.0) * step_height_;
                        next_pos.x *= -1.0;
                        next_pos.y *= -1.0;
                        next_pos.yaw *= -1.0;
                        rapidMove(next_pos, active_legs, false);
                    }
                    break;
                case StepType::RETURN_TO_NEUTRAL:
                    {
                        uint8_t step_group;
                        if ((last_step_progress_ < 0.5 && step_progress >= 0.5) || (last_step_progress_ >= 0.5 && step_progress < 0.5)) {
                            // Switch step groups
                            if (step_progress < 0.5) {
                                switch(last_step_type_) {
                                    case StepType::GROUP0:
                                    case StepType::GROUP1:
                                        step_group = static_cast<uint8_t>(last_step_type_);
                                        break;
                                    default:
                                        step_group = 0;
                                        break;
                                }
                            }
                            else {
                                // Since return to neutral doesn't move the body, we reset the position when moving switching which legs are moving
                                // to neutral as the opposite set of legs is in the reverse position as the other set of legs
                                start_pos_.x *= -1.0;
                                start_pos_.y *= -1.0;
                                start_pos_.yaw *= -1.0;
                                switch(last_step_type_) {
                                    case StepType::GROUP0:
                                    case StepType::GROUP1:
                                        step_group = static_cast<uint8_t>(last_step_type_) ^ 1;
                                        break;
                                    default:
                                        step_group = 1;
                                }
                            }
                        }
                        else {
                            if (step_progress < 0.5) {
                                switch(last_step_type_) {
                                    case StepType::GROUP0:
                                    case StepType::GROUP1:
                                        step_group = static_cast<uint8_t>(last_step_type_);
                                        break;
                                    default:
                                        step_group = 0;
                                        break;
                                }
                            }
                            else {
                                // Since return to neutral doesn't move the body, we reset the position when moving switching which legs are moving
                                // to neutral as the opposite set of legs is in the reverse position as the other set of legs
                                switch(last_step_type_) {
                                    case StepType::GROUP0:
                                    case StepType::GROUP1:
                                        step_group = static_cast<uint8_t>(last_step_type_) ^ 1;
                                        break;
                                    default:
                                        step_group = 1;
                                }
                            }
                        }
                        std::array<bool, NUM_LEGS> active_legs;
                        for (uint8_t i = 0; i < NUM_LEGS / NUM_STEP_GROUPS; i++) {
                            active_legs[step_groups_[step_group][i]] = true;
                            active_legs[step_groups_[(step_group^1)][i]] = false;
                        }
                        double adjusted_step_progress = step_progress < 0.5 ? 2.0 * step_progress : (step_progress - 0.5) * 2.0;
                        next_pos = (end_pos_ - start_pos_) * adjusted_step_progress + start_pos_;
                        next_pos.z += -4 * adjusted_step_progress * (adjusted_step_progress - 1.0) * step_height_;
                        rapidMove(next_pos, active_legs, true);
                    }
                break;
                case StepType::LEG_5_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_4_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_3_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_2_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_1_LINEAR_MOVE_ABSOLUTE:
                    {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Running single leg step of type %d", static_cast<uint8_t>(current_step_type_));
                        std::array<bool, NUM_LEGS> active_legs;
                        std::fill(active_legs.begin(), active_legs.end(), false);
                        uint8_t leg_num = static_cast<uint8_t>(current_step_type_) - static_cast<uint8_t>(StepType::LEG_0_LINEAR_MOVE_ABSOLUTE);
                        active_legs[leg_num] = true;
                        next_pos = (end_pos_ - start_pos_) * step_progress + start_pos_;
                        rapidMove(next_pos, active_legs, true);
                    }
                break;
                case StepType::LINEAR_MOVE_RELATIVE:
                case StepType::LINEAR_MOVE_ABSOLUTE:
                    next_pos = (end_pos_ - start_pos_) * step_progress + start_pos_;			
                    rapidMove(next_pos);
                break;
                case StepType::RAPID_MOVE:
                    rapidMove(end_pos_);
                default:
                    break;
           }
        }
    } else {
        // if there is a step to take
        if (step_queue_.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
            "enqueing step based on v_command x:%f; y:%f; z:%f; roll:%f; pitch:%f; yaw:%f;", 
            v_command.x, v_command.y, v_command.z, v_command.roll, v_command.pitch, v_command.yaw);
            enqueueMaxStepInDirection_(v_command.unitVector(), std::max(fabs(v_command.scaledMagnitude()) / max_step_speed_, 0.25));
        }
        else {
            last_step_type_ = current_step_type_;
			current_step_type_ = step_queue_.front()->step_type;
			if (current_step_type_ == StepType::GROUP0) {
				if (last_step_type_ == StepType::GROUP1) {
					current_pos_.x *= -1.0;
					current_pos_.y *= -1.0;
					current_pos_.yaw *= -1.0;
				}
			}
			else if (current_step_type_ == StepType::GROUP1) {
				if (last_step_type_ == StepType::GROUP0) {
					current_pos_.x *= -1.0;
					current_pos_.y *= -1.0;
					current_pos_.yaw *= -1.0;
				}
			}
			
			switch(current_step_type_) {
				case StepType::RETURN_TO_NEUTRAL:
				case StepType::RAPID_MOVE:
				case StepType::LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_5_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_4_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_3_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_2_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_1_LINEAR_MOVE_ABSOLUTE:
                case StepType::LEG_0_LINEAR_MOVE_ABSOLUTE:
					end_pos_ = step_queue_.front()->end_pos;
					break;
				case StepType::GROUP0:
				case StepType::GROUP1:
				case StepType::LINEAR_MOVE_RELATIVE:
				default:
					end_pos_ = current_pos_ + step_queue_.front()->end_pos;
					break;
			}
			start_pos_ = current_pos_;
			move_time_ = step_queue_.front()->time;
			move_start_time_ = current_time;
			step_queue_.dequeue();
			step_in_progress_ = true;
            RCLCPP_INFO(get_logger(), "Running step with:\n    start_pos: %f %f %f %f %f %f\n    end_pos: %f %f %f %f %f %f\n    move_time: %f\n    move_start_time: %f",
                start_pos_.x,
                start_pos_.y,
                start_pos_.z, 
                start_pos_.roll,
                start_pos_.pitch,
                start_pos_.yaw,
                end_pos_.x,
                end_pos_.y,
                end_pos_.z, 
                end_pos_.roll,
                end_pos_.pitch,
                end_pos_.yaw,
                move_time_.seconds(),
                move_start_time_.seconds()
            );
            
		}
    }
}

void TripodGaitNode::rapidMove(Pose6D pos) {
    std::array<bool, NUM_LEGS> active_legs;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        active_legs[i] = true;
    }
    rapidMove(pos, active_legs, true);
}

void TripodGaitNode::rapidMove(Pose6D pos, std::array<bool, NUM_LEGS> active_legs, bool update_pos) {
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (active_legs[i]) {
            hexapod_msgs::msg::BodyPose body_pose;
            body_pose.x = pos.x;
            body_pose.y = pos.y;
            body_pose.z = pos.z;
            body_pose.roll = pos.roll;
            body_pose.pitch = pos.pitch;
            body_pose.yaw = pos.yaw;
            body_pose.leg_number = i;
            publishBodyPose(body_pose);
        }
    }
    if (update_pos)
    {
        current_pos_ = pos;
    }
}

// StepType TripodGaitNode::getNextStepType_(Pose6D direction_vector) {
//     if (current_step_type_ == StepType::RETURN_TO_NEUTRAL) {
//         return StepType::GROUP0; // If just returned to neutral then either leg group is fine
//     }
    
//     double no_flip_magnitude = _getMaxStepMagnitudeInDirection(Pose6D direction_vector, false);
//     double with_flip_magnitude = _getMaxStepMagnitudeInDirection(Pose6D direction_vector, false);

//     if (no_flip_magnitude > with_flip_magnitude) {
//         if (no_flip_magnitude > 0.5 * max_step_length_) {
//             return StepType::RETURN_TO_NEUTRAL
//         }
//         return current_step_type_;
//     }
//     else {
//         if (with_flip_magnitude > 0.5 * max_step_length_) {
//             return StepType::RETURN_TO_NEUTRAL
//         }
//         return current_step_type_ ^ 0x01;
//     }
// }

double TripodGaitNode::getMaxStepMagnitude_() {
	// Pose6D current_pos = step_queue_.getCurrentQueueEndPos();
	return max_step_length_;
}

double TripodGaitNode::getMaxStepMagnitudeInDirection_(Pose6D direction_vector, bool flipped_step_group) {	
	// Create a vector of the next step that ends up with the max step magnitude in the direction of the move
	// and enqueue it

	// Constraints:
	// 1. The step must be in the direction of the move
	// 2. The step should end up in a position that is equal to the max step magnitude
	// | step_queue_.getCurrentQueueEndPos() + step | = MAX_STEP_MAGNITUDE
	// 3. The step should be in the direction of the move
	
	Pose6D buffer1 = step_queue_.getCurrentQueueEndPos();
	buffer1.z = 0.00; // For now we don't consider Z, roll, or pitch
	buffer1.roll = 0.00;
	buffer1.pitch = 0.00;
    buffer1.yaw *= ROTATION_MAGNITUDE_SCALE;
	if (flipped_step_group) {
		buffer1 *= -1.0; // If the step group has been flipped, then the previous step was in the opposite direction
	}
	
	Pose6D buffer2 = direction_vector.unitVector();
	buffer2.z = 0.00; // For now we don't consider Z, roll, or pitch
	buffer2.roll = 0.00;
	buffer2.pitch = 0.00;
	// buffer2.yaw *= ROTATION_MAGNITUDE_SCALE; // Scale yaw to have a similar range as x and y

	double c = pow(buffer1.x, 2) + pow(buffer1.y, 2) + pow(buffer1.yaw, 2) - pow(getMaxStepMagnitude_(), 2);
	double b = 2.0 * (buffer1.x * buffer2.x + buffer1.y * buffer2.y + buffer1.yaw * buffer2.yaw);
	double a = pow(buffer2.x, 2) + pow(buffer2.y, 2) + pow(buffer2.yaw, 2);

	// Solve the quadratic equation for the step magnitude
	double discriminant = pow(b, 2) - 4.0 * a * c;
	double step_magnitude;
	if (discriminant < 0.0) {
		// No solution, return 0
		step_magnitude = 0.00;
	}
	else if (fabs(discriminant) <= 0.001) {
		// One solution, use it
		step_magnitude = -b / (2.0 * a);
	}
	else {
		// Two solutions, use the positive one (if there is one)
		step_magnitude = ((-b + sqrt(discriminant)) / (2.0 * a)) > 0.00 ? (-b + sqrt(discriminant)) / (2.0 * a) : (-b - sqrt(discriminant)) / (2.0 * a);
	}
	return step_magnitude;
}