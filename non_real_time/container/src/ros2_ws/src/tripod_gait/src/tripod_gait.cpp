#include "tripod_gait.hpp"

TripodGaitNode::TripodGaitNode()
    : Gait("tripod_gait")
{
}



void TripodGaitNode::updateGait(
    double dt, double current_time)
{
    hexapod_msgs::msg::FootTargetArray foot_targets;
    hexapod_msgs::msg::BodyPose next_body_pose;
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
           switch(_current_step_type) {
                case StepType::GROUP0:
                case StepType::GROUP1:
                    next_pos = (end_pos_ - start_pos_) * step_progress + start_pos_;
					next_body_pose = next_pos.toBodyPose();
                    
                    step_group = static_cast<uint8_t>(_current_step_type);
					std::array<uint8_t, NUM_LEGS / 2> lifted_legs = _step_groups[step_group];
                    std::array<uint8_t, NUM_LEGS / 2> grounded_legs = _step_groups[step_group ^ 1];

                    Pose3D lift_leg_correction = Pose3D(next_body_pose);
                    lift_leg_correction.x *= -2.0; //Reverse and double to go in the opposite direction the same amount
					lift_leg_correction.y *= -2.0;
					// lift_leg_correction.yaw *= -2.0; //TODO: Figure out yaw moves in this paradigm
                    
                    lift_leg_correction.z += -4 * step_progress * (step_progress - 1.0) * _step_height;
                    break;
                case StepType::RETURN_TO_NEUTRAL:
                    // Compute foot targets for return to neutral based on step_progress
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
        double v_command_magnitude = v_command.magnitude();
        if (v_command_magnitude > 0.1) {
            start_pos_ = _current_pos;
            
            step_length = v_command_magnitude * (1.0 / max_step_speed_) * max_step_length_;

            move_start_time_ = current_time;
            move_end_time_ = step_length / v_command_magnitude + move_start_time_;
            end_pos_ = step_length * v_command.unitVector();
            step_in_progress_ = true;
            current_step_type_ = getNextStepType();

            if (current_step_type_ == StepType::GROUP0) {
				if (last_step_type_ == StepType::GROUP1) {
					_current_pos.x *= -1.0;
					_current_pos.y *= -1.0;
					_current_pos.yaw *= -1.0;
				}
			}
			else if (current_step_type_ == StepType::GROUP1) {
				if (last_step_type_ == StepType::GROUP0) {
					_current_pos.x *= -1.0;
					_current_pos.y *= -1.0;
					_current_pos.yaw *= -1.0;
				}
			}
        }
    }
    publishFootTargets(foot_targets);
    publishBodyPose(pose);
}

StepType getNextStepType(Pose6D direction_vector) {
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

double TripodGait::_getMaxStepMagnitudeInDirection(Pose6D direction_vector, bool flipped_step_group) {	
	// Create a vector of the next step that ends up with the max step magnitude in the direction of the move
	// and enqueue it

	// Constraints:
	// 1. The step must be in the direction of the move
	// 2. The step should end up in a position that is equal to the max step magnitude
	// | _step_queue.getCurrentQueueEndPos() + step | = MAX_STEP_MAGNITUDE
	// 3. The step should be in the direction of the move
	
	buffer1 = Pose6D(current_pos_);
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