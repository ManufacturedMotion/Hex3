#pragma once

#include "gait.hpp"
#include "pose.hpp"
#include "step_queue.hpp"
#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose.hpp"
#include <array>

#define ROTATION_MAGNITUDE_SCALE 100.0
#define NUM_LEGS 6
#define NUM_STEP_GROUPS 2

class TripodGaitNode : public Gait
{
public:
    TripodGaitNode());

private:
    void updateGait(
        double dt, rclcpp::Duration current_time) override;
    rclcpp::Duration enqueueMaxStepInDirection_(Pose6D direction_vector, double scalar);
    double getMaxStepMagnitude_();
    double getMaxStepMagnitudeInDirection_(Pose6D direction_vector, bool flipped_step_group);
    void rapidMove(Pose6D pos, std::array<bool, NUM_LEGS> active_legs);

    std::array<std::array<uint8_t, 3>, 2> step_groups_ = {{
        {{0, 3, 4}}, // Group 0: Legs 1, 4, 5
        {{1, 2, 5}}  // Group 1: Legs 2, 3, 6
    }};

    double step_height_ = 40.0;
    double max_step_length_ = 100.0;
    double max_step_speed_ = 200.0;
    double neutral_z_ = 180.0;

    double last_step_progress_ = 0.0;

    bool step_in_progress_ = false;
    rclcpp::Duration move_start_time_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Duration move_time_ = rclcpp::Duration::from_nanoseconds(0);
    StepType current_step_type_ = StepType::NONE;
    StepType last_step_type_ = StepType::NONE;

    Pose6D start_pos_;
    Pose6D end_pos_;
    Pose6D current_pos_;

    StepType getNextStepType();

    StepQueue step_queue_;

};