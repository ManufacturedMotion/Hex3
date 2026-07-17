#pragma once

#include "gait.hpp"
#include "pose.hpp"
#include "step_queue.hpp"
#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose.hpp"
#include "constants.hpp"
#include <array>

#define NUM_LEGS 6
#define NUM_STEP_GROUPS 2

enum class MacroCode {
    STAND = 1,
    SIT = 2,
    FLIP = 4,
    WAVE = 8
};

class TripodGaitNode : public Gait
{
public:
    TripodGaitNode();

private:
    void updateGait(
        double dt, rclcpp::Duration current_time) override;

    void runMacro(int8_t macro_num) override;
    rclcpp::Duration enqueueMaxStepInDirection_(Pose6D direction_vector, double scalar);
    double getMaxStepMagnitude_();
    double getMaxStepMagnitudeInDirection_(Pose6D direction_vector, bool flipped_step_group);
    void rapidMove(Pose6D pos, std::array<bool, NUM_LEGS> active_legs, bool update_pos);
    void rapidMove(Pose6D pos);
    std::array<std::array<uint8_t, 3>, 2> step_groups_ = {{
        {{0, 2, 4}}, // Group 0: Legs 1, 4, 5
        {{1, 3, 5}}  // Group 1: Legs 2, 3, 6
    }};

    double step_height_ = STEP_HEIGHT;
    double max_step_length_ = MAX_STEP_MAGNITUDE;
    double max_step_speed_ = MAX_SPEED;

    double last_step_progress_ = 0.0;

    void legRapidMove(uint8_t leg_num, Pose6D pos);

    bool step_in_progress_ = false;
    rclcpp::Duration move_start_time_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Duration move_time_ = rclcpp::Duration::from_nanoseconds(0);
    StepType current_step_type_ = StepType::NONE;
    StepType last_step_type_ = StepType::NONE;
    StepType next_step_type_ = StepType::GROUP0;

    Pose6D start_pos_;
    Pose6D end_pos_;
    Pose6D current_pos_;

    StepType getNextStepType();

    StepQueue step_queue_ = StepQueue(this->get_logger());
};