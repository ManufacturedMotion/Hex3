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

StepType getNextStepType() {
    if (current_step_ == StepType::GROUP0)
}