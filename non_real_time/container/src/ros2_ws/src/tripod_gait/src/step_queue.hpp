#pragma once

#include <deque>
#include <cstdint>

#include "pose.hpp"
#include "rclcpp/rclcpp.hpp"

DEFAULT_MOVE_SPEED = 100.0

enum class StepType {
    GROUP0 = 0,
    GROUP1 = 1,
    LINEAR_MOVE_RELATIVE = 254,
    LINEAR_MOVE_ABSOLUTE = 253,
    RETURN_TO_NEUTRAL = 252,
    RAPID_MOVE = 251,
    NONE = 255,
};

enum class StepQueueState {
    NEUTRAL = 0,
    STEPPED = 1,
    UNINITIALIZED = 2,
};

struct Step
{
    Pose6D end_pos;
    double speed = DEFAULT_MOVE_SPEED;
    StepType step_type = StepType::RAPID_MOVE;
    rclcpp::Duration time = rclcpp::Duration::from_nanoseconds(0);

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
    StepType last_step_type_ = StepType::NONE;
};

