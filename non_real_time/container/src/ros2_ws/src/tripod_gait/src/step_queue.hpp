#pragma once

#include <deque>
#include <cstdint>

#include "pose.hpp"
#include "rclcpp/rclcpp.hpp"

class StepQueue
{
public:
    StepQueue() = default;

    rclcpp::Duration enqueue(
        const Pose6D& end_pos,
        double speed,
        StepType step_type);

    bool dequeue();

    bool empty() const
    {
        return queue_.empty();
    }

    std::size_t size() const
    {
        return queue_.size();
    }

    const Step* front() const
    {
        return queue_.empty() ? nullptr : &queue_.front();
    }

    Pose6D getCurrentQueueEndPos() const
    {
        return end_pos_;
    }

private:
    std::deque<Step> queue_;

    Pose6D end_pos_;

    StepQueueState state_ = StepQueueState::UNINITIALIZED;
    StepType last_step_type_ = StepType::RAPID_MOVE;
};

struct Step
{
    Pose6D end_pos;
    double speed = DEFAULT_MOVE_SPEED;
    StepType step_type = StepType::RAPID_MOVE;
    rclcpp::Duration time = 0.0;

    Step() = default;

    Step(
        const Pose6D& pos,
        double spd,
        StepType type,
        rclcpp::Duration duration)
        : end_pos(pos)
        , speed(spd)
        , step_type(type)
        , time(duration)
    {
    }
};