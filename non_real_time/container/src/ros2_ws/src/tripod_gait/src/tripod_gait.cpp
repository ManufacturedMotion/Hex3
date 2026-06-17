#include "tripod_gait.hpp"

TripodGaitNode::TripodGaitNode()
    : Gait("tripod_gait")
{
}

uint32_t TripodGaitNode::enqueueMaxStepInDirection_(Pose6D direction_vector, double scalar) {
	direction_vector.z = 0.00; // For now we don't consider Z, roll, or pitch
	direction_vector.roll = 0.00;
	direction_vector.pitch = 0.00;
	if (direction_vector.magnitude() < 0.001) {
		return 0; // No step to take
	}

	double speed = v_command.magnitude();
	uint32_t walk_time = 0;
	double max_step_magnitude_with_flip = _getMaxStepMagnitudeInDirection(direction_vector, true);
	double max_step_magnitude_without_flip = _getMaxStepMagnitudeInDirection(direction_vector, false);
	double max_step_magnitude = max_step_magnitude_without_flip;
	if (max_step_magnitude_with_flip > max_step_magnitude_without_flip) {
		_next_step_type = static_cast<decltype(_next_step_type)>(static_cast<uint8_t>(_next_step_type) ^ 1);
		max_step_magnitude = max_step_magnitude_with_flip;
	}

	if (max_step_magnitude < get_max_step_magnitude()) {
		Position buffer0;
		buffer0.setPos(_step_queue.getCurrentQueueEndPos());
		buffer0.x = 0.00;
		buffer0.y = 0.00;
		buffer0.yaw = 0.00;
		walk_time += _step_queue.enqueue(buffer0, speed, StepType::RETURN_TO_NEUTRAL);
	}
	
	Position step_vector = direction_vector.unitVector() * max_step_magnitude;
	walk_time += _step_queue.enqueue(step_vector * fabs(scalar), speed, _next_step_type);
	return walk_time;
}


void TripodGaitNode::updateGait(
    double dt, double current_time)
{
    hexapod_msgs::msg::FootTargetArray foot_targets;
    hexapod_msgs::msg::BodyPoseArray next_body_poses;
    hexapod_msgs::msg::BodyPose base_body_pose = current_pos_.toBodyPose();
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        next_body_poses.body_poses[i] = base_body_pose;
    }
    //
    // tripod gait logic here
    //
    if (step_in_progress_) {
        double step_progress = (current_time - static_cast<double>(move_start_time_)) / gait_period_;
        Pose6D next_pos;
        if (step_progress >= 1.0) {
            // Step complete
            step_in_progress_ = false;
        } else {
           switch(current_step_type_) {
                case StepType::GROUP0:
                case StepType::GROUP1:
                    next_pos = (end_pos_ - start_pos_) * step_progress + start_pos_;
                    base_body_pose = next_pos.toBodyPose();
                    for (uint8_t i = 0; i < NUM_LEGS; i++) {
                        next_body_poses.body_poses[i] = base_body_pose;
                    }
                    
                    step_group = static_cast<uint8_t>(current_step_type_);
					std::array<uint8_t, NUM_LEGS / 2> lifted_legs = step_groups_[step_group];

                    Pose3D lift_leg_correction = Pose3D(base_body_pose);
                    lift_leg_correction.x *= -2.0; //Reverse and double to go in the opposite direction the same amount
					lift_leg_correction.y *= -2.0;
					// lift_leg_correction.yaw *= -2.0; //TODO: Figure out yaw moves in this paradigm
                    
                    lift_leg_correction.z += -4 * step_progress * (step_progress - 1.0) * _step_height;

                    for (uint8_t leg : lifted_legs) {
                        hexapod_msgs::msg::BodyPose& leg_pose = next_body_poses.body_poses[leg];
                        leg_pose.x = base_body_pose.x + static_cast<float>(lift_leg_correction.x);
                        leg_pose.y = base_body_pose.y + static_cast<float>(lift_leg_correction.y);
                        leg_pose.z = base_body_pose.z + static_cast<float>(lift_leg_correction.z);
                    }
                    break;
                case StepType::RETURN_TO_NEUTRAL:
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
                            switch(_last_step_type) {
                                case StepType::GROUP0:
                                case StepType::GROUP1:
                                    step_group = static_cast<uint8_t>(_last_step_type) ^ 1;
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
                    
                    for (uint8_t i = 0; i < NUM_LEGS / NUM_STEP_GROUPS; i++) {
                        active_legs[step_groups_[step_group][i]] = true;
                        active_legs[step_groups_[(step_group^1)][i]] = false;
                    }
                    adjusted_step_progress = step_progress < 0.5 ? 2.0 * step_progress : (step_progress - 0.5) * 2.0;
                    next_pos = (end_pos_ - start_pos_) * adjusted_step_progress + start_pos_;
                    next_pos.z -= -4 * adjusted_step_progress * (adjusted_step_progress - 1.0) * step_height_;
                    rapidMove(next_pos, active_legs, true);
                break;
                case StepType::LINEAR_MOVE_RELATIVE:
                    // Compute foot targets for linear move relative based on step_progress
                    break;
                case StepType::LINEAR_MOVE_ABSOLUTE:
                    // Compute foot targets for linear move absolute based on step_progress
                    break;
                case StepType::RAPID_MOVE:
                    // Compute foot targets for rapid move based on step_progress
                    break;
                default:
                    break;
           }
        }
    } else {
        // if there is a step to take
        if (step_queue_.empty()) {
            _enqueueMaxStepInDirection(v_command, max(fabs(v_command.magnitude()) / max_step_speed_, 0.25));
        }
        else {
            _last_step_type = _current_step_type;
			_current_step_type = _step_queue.head->step_type;
			if (_current_step_type == StepType::GROUP0) {
				if (_last_step_type == StepType::GROUP1) {
					current_pos_.x *= -1.0;
					current_pos_.y *= -1.0;
					current_pos_.yaw *= -1.0;
				}_
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
					end_pos_ = step_queue_.head->end_pos;
					break;
				case StepType::GROUP0:
				case StepType::GROUP1:
				case StepType::LINEAR_MOVE_RELATIVE:
				default:
					end_pos_ = current_pos_ + step_queue_.head->end_pos;
					break;
			}
			start_pos_ = current_pos_;
			current_speed_ = step_queue_.head->speed;
			move_time_ = step_queue_.head->time;
			move_start_time_ = current_time;
			step_queue_.dequeue();
			step_in_progress_ = true;
		}
    }

    publishFootTargets(foot_targets);
    publishBodyPoses(next_body_poses);
}

StepType TripodGaitNode::getNextStepType_(Pose6D direction_vector) {
    if (current_step_type_ == StepType::RETURN_TO_NEUTRAL) {
        return StepType::GROUP0; // If just returned to neutral then either leg group is fine
    }
    
    double no_flip_magnitude = _getMaxStepMagnitudeInDirection(Pose6D direction_vector, false);
    double with_flip_magnitude = _getMaxStepMagnitudeInDirection(Pose6D direction_vector, false);

    if (no_flip_magnitude > with_flip_magnitude) {
        if (no_flip_magnitude > 0.5 * max_step_length_) {
            return StepType::RETURN_TO_NEUTRAL
        }
        return current_step_type_;
    }
    else {
        if (with_flip_magnitude > 0.5 * max_step_length_) {
            return StepType::RETURN_TO_NEUTRAL
        }
        return current_step_type_ ^ 0x01;
    }
}

double TripodGaitNode::getMaxStepMagnitude_() {
	Position current_pos = _step_queue.getCurrentQueueEndPos();
	return MAX_STEP_MAGNITUDE - sqrt(pow((current_pos.z - 100.0) / 2.0, 2) + pow(current_pos.roll, 2) + pow(current_pos.pitch, 2)) / 2.0;
}

double TripodGaitNode::getMaxStepMagnitudeInDirection_(Pose6D direction_vector, bool flipped_step_group) {	
	// Create a vector of the next step that ends up with the max step magnitude in the direction of the move
	// and enqueue it

	// Constraints:
	// 1. The step must be in the direction of the move
	// 2. The step should end up in a position that is equal to the max step magnitude
	// | _step_queue.getCurrentQueueEndPos() + step | = MAX_STEP_MAGNITUDE
	// 3. The step should be in the direction of the move
	
	buffer1 = step_queue.getCurrenQueueEndPos();
	buffer1.z = 0.00; // For now we don't consider Z, roll, or pitch
	buffer1.roll = 0.00;
	buffer1.pitch = 0.00;
	if (flipped_step_group) {
		buffer1 *= -1.0; // If the step group has been flipped, then the previous step was in the opposite direction
	}
	
	Pose6D buffer2 = direction_vector.unitVector();
	buffer2.z = 0.00; // For now we don't consider Z, roll, or pitch
	buffer2.roll = 0.00;
	buffer2.pitch = 0.00;
	buffer2.yaw *= ROTATION_MAGNITUDE_SCALE; // Scale yaw to have a similar range as x and y

	double c = pow(buffer1.x, 2) + pow(buffer1.y, 2) + pow(buffer1.yaw, 2) - pow(get_max_step_magnitude(), 2);
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