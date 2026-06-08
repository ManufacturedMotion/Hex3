#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "hexapod_msgs/msg/foot_target.hpp"
#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose.hpp"

using namespace std::chrono_literals;

class TesterNode : public rclcpp::Node
{
public:
    TesterNode() : Node("hexapod_tester")
    {
        foot_pub_ = this->create_publisher<
            hexapod_msgs::msg::FootTargetArray>(
            "/foot_targets", 10);

        body_pub_ = this->create_publisher<
            hexapod_msgs::msg::BodyPose>(
            "/body_pose", 10);

        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&TesterNode::update, this));

        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "Hexapod tester node started");
    }

private:

    void update()
    {
        double t =
            (this->now() - start_time_).seconds();

        publishBodyPose(t);
        publishFootTargets(t);
    }

    void publishBodyPose(double t)
    {
        hexapod_msgs::msg::BodyPose msg;

        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = -200.0 + 40.0 * std::sin(2.0 * t);
        msg.roll  = 0.0;
        msg.pitch = 0.0;
        msg.yaw   = 0.0;

        body_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,   // 1 Hz log rate
            "Published BodyPose: x=%.2f y=%.2f z=%.2f roll=%.3f pitch=%.3f yaw=%.3f",
            msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw
        );
    }

    void publishFootTargets(double t)
    {
        hexapod_msgs::msg::FootTargetArray msg;


        for (int i = 0; i < 6; i++)
        {
            double phase = t + i * 0.5;

            hexapod_msgs::msg::FootTarget ft;

            ft.x = 0.0; //100.0 * std::cos(phase);
            ft.y = 0.0;
            ft.z = 0.0; // (std::sin(phase) > 0.0) ? 40.0 : 0.0;

            msg.foot_targets[i] = ft;
        }

        foot_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,   // 1 Hz log rate
            "Published FootTargets (6 legs) sample: leg0=(%.1f, %.1f, %.1f)",
            msg.foot_targets[0].x,
            msg.foot_targets[0].y,
            msg.foot_targets[0].z
        );
    }

private:
    rclcpp::Publisher<
        hexapod_msgs::msg::FootTargetArray>::SharedPtr foot_pub_;

    rclcpp::Publisher<
        hexapod_msgs::msg::BodyPose>::SharedPtr body_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TesterNode>());
    rclcpp::shutdown();
    return 0;
}