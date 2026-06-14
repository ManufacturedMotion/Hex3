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
            10ms,
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
        msg.z = -200.0; // + 40.0 * std::sin(2.0 * t);
        msg.roll  = 0.0;
        msg.pitch = 0.0;
        msg.yaw   = 0.0;

        body_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Published BodyPose: x=%.2f y=%.2f z=%.2f roll=%.3f pitch=%.3f yaw=%.3f",
            msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw
        );
    }

    void publishFootTargets(double t)
    {
        hexapod_msgs::msg::FootTargetArray msg;

        const double step_freq = 1.2;        // gait speed
        const double step_length = 60.0;     // mm
        const double lift_height = 35.0;     // mm

        // Tripod grouping
        const int tripodA[3] = {1, 2, 3};
        const int tripodB[3] = {4, 5, 6};

        auto compute_leg = [&](int leg_id, bool in_tripod_a)
        {
            double leg_phase_offset = in_tripod_a ? 0.0 : 0.5;

            double phase = fmod(t * step_freq + leg_phase_offset, 1.0);

            hexapod_msgs::msg::FootTarget ft;

            // Default stance position (you likely want real neutral foot positions here)
            double x0 = 0.0;
            double y0 = 0.0;
            double z0 = 0.0;

            if (phase < 0.5)
            {
                // STANCE: move foot backward relative to body motion
                double p = phase / 0.5; // 0→1

                ft.x = x0 - step_length * (p - 0.5); // backward sweep
                ft.z = z0;
            }
            else
            {
                // SWING: lift and move forward
                double p = (phase - 0.5) / 0.5; // 0→1

                ft.x = x0 + step_length * (p - 0.5);
                ft.z = z0 + lift_height * std::sin(M_PI * p);
            }

            ft.y = y0;
            return ft;
        };

        // Fill tripod A
        for (int i = 0; i < 3; i++)
        {
            int leg = tripodA[i];
            msg.foot_targets[leg] = compute_leg(leg, true);
        }

        // Fill tripod B
        for (int i = 0; i < 3; i++)
        {
            int leg = tripodB[i];
            msg.foot_targets[leg] = compute_leg(leg, false);
        }

        foot_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Tripod gait published: L0(%.1f, %.1f, %.1f)",
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