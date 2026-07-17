#include "step_queue.hpp"

StepQueue::StepQueue(rclcpp::Logger logger) : logger_(logger){
}

rclcpp::Duration StepQueue::enqueue(
    Pose6D op_end_pos,
    double op_speed,
    StepType op_step_type)
{
    
    if (state_ == StepQueueState::UNINITIALIZED &&
        op_step_type != StepType::RAPID_MOVE)
    {
        return rclcpp::Duration::from_nanoseconds(0);
    }

    rclcpp::Duration op_time = rclcpp::Duration::from_nanoseconds(0);

    switch (op_step_type)
    {
        case StepType::GROUP0:
        case StepType::GROUP1:
        case StepType::GROUP2:
        {
            if (!(state_ == StepQueueState::NEUTRAL ||
                  last_step_type_ == op_step_type)) // If switching groups from a non neutral position, then flip X, Y, and Yaw
            {
                end_pos_.x *= -1.0;
                end_pos_.y *= -1.0;
                end_pos_.yaw *= -1.0;
            }

            end_pos_ += op_end_pos;
            op_time = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>((op_end_pos.scaledMagnitude() / op_speed) * 1000000000.0));

            state_ = StepQueueState::STEPPED;
            break;
        }

        case StepType::LINEAR_MOVE_RELATIVE:
        {
            end_pos_ += op_end_pos;

            op_time = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>((op_end_pos.scaledMagnitude() / op_speed) * 1000000000.0));
            break;
        }

        case StepType::LINEAR_MOVE_ABSOLUTE:
        {
            op_time = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(((op_end_pos - end_pos_).scaledMagnitude() /
                 op_speed) * 1000000000.0));

            if (op_time.seconds() < 0.001)
            {
                return rclcpp::Duration::from_nanoseconds(0);
            }

            end_pos_ = op_end_pos;
            break;
        }

        case StepType::RETURN_TO_NEUTRAL:
        {
            if (state_ == StepQueueState::NEUTRAL)
            {
                return rclcpp::Duration::from_nanoseconds(0);
            }

            op_time = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(((op_end_pos - end_pos_).scaledMagnitude() /
                 op_speed) * 2000000000.0));

            end_pos_ = op_end_pos;
            state_ = StepQueueState::NEUTRAL;
            break;
        }

        case StepType::RAPID_MOVE:
        {
            op_time = rclcpp::Duration::from_nanoseconds(100000000);

            end_pos_ = op_end_pos;
            state_ = StepQueueState::NEUTRAL;
            break;
        }

        case StepType::LEG_5_LINEAR_MOVE_ABSOLUTE:
        case StepType::LEG_4_LINEAR_MOVE_ABSOLUTE:
        case StepType::LEG_3_LINEAR_MOVE_ABSOLUTE:
        case StepType::LEG_2_LINEAR_MOVE_ABSOLUTE:
        case StepType::LEG_1_LINEAR_MOVE_ABSOLUTE:
        case StepType::LEG_0_LINEAR_MOVE_ABSOLUTE:
        {
            op_time = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(((op_end_pos - end_pos_).scaledMagnitude() /
                 op_speed) * 1000000000.0));

            if (op_time.seconds() < 0.001)
            {
                return rclcpp::Duration::from_nanoseconds(0);
            }

            end_pos_ = op_end_pos;
            break;
        }
        
        default:
            return rclcpp::Duration::from_nanoseconds(0);
    }

    queue_.emplace_back(
        op_end_pos,
        op_speed,
        op_step_type,
        op_time);

    last_step_type_ = op_step_type;
    RCLCPP_INFO(logger_, "Enqueued step of type %d, which will take %f seconds; new queue length: %ld", static_cast<uint8_t>(op_step_type), op_time.seconds(), size());

    return op_time;
}

bool StepQueue::dequeue()
{
    if (queue_.empty())
    {
        return false;
    }

    queue_.pop_front();
    RCLCPP_INFO(logger_, "Dequeued step successfully");
    return true;
}