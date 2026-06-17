#pragma once

#include <deque>
#include <cstdint>

class StepQueue
{
public:
    StepQueue() = default;

    uint32_t enqueue(
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