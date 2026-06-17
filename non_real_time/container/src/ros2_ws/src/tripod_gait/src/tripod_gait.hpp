#pragma once

#include "gait.hpp"
#include "pose.hpp"
#include "step_queue.hpp"
#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose.hpp"
#include <array>

#define ROTATION_MAGNITUDE_SCALE 100.0

enum class StepType {
    GROUP0 = 0,
    GROUP1 = 1,
    LINEAR_MOVE_RELATIVE = 254,
    LINEAR_MOVE_ABSOLUTE = 253,
    RETURN_TO_NEUTRAL = 252,
    RAPID_MOVE = 251,
    NONE = 255,
};

class TripodGaitNode : public Gait
{
public:
    TripodGaitNode();

private:
    void updateGait(
        double dt, rclcpp::Time current_time) override;
    rclcpp::Duration enqueueMaxStepInDirection_(Pose6D direction_vector, double scalar);
    double getMaxStepMagnitude_();
    double getMaxStepMagnitudeInDirection_(Pose6D direction_vector, bool flipped_step_group);

    std::array<std::array<uint8_t, 3>, 2> step_groups_ = {
        {0, 3, 4}, // Group 0: Legs 1, 4, 5
        {1, 2, 5}  // Group 1: Legs 2, 3, 6
    };
    double phase_ = 0.0;
    double gait_period_ = 0.5;

    double step_height_ = 40.0;
    double max_step_length_ = 100.0;
    double max_step_speed_ = 200.0;
    double neutral_z_ = 180.0;

    double last_step_progress_ = 0.0;

    bool step_in_progress_ = false;
    rclcpp::Time move_start_time_ = 0.0;
    rclcpp::Time move_end_time_ = 0.0;
    StepType current_step_type_ = StepType::NONE;
    StepType last_step_type_ = StepType::NONE;

    Pose6D start_pos_;
    Pose6D end_pos_;
    Pose6D current_pos_;

    StepType getNextStepType();

    StepQueue step_queue_;

};