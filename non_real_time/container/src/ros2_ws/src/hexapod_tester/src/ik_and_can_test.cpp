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
    double body_x_ = 0.0;
    double body_y_ = 0.0;
    double body_yaw_ = 0.0;

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

        const double speed = 20.0;   // mm/s forward
        const double bounce = 10.0;  // mm

        body_x_ = speed * t;

        msg.x = body_x_;
        msg.y = 0.0;
        msg.z = -200.0 + bounce * std::sin(2.0 * M_PI * 1.0 * t);

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

        const double step_freq   = 1.2;
        const double step_length = 50.0;   // forward stride in mm
        const double lift_height = 30.0;

        // Tripod split (classic alternating diagonals)
        const int tripodA[3] = {0, 3, 4};
        const int tripodB[3] = {1, 2, 5};

        // These are your FIXED nominal foot placements in BODY frame
        // You SHOULD tune these to your real geometry
        double nominal_x[6] = { 100,  100,  0, -100, -100,  0 };
        double nominal_y[6] = { 120,   40, 120,  120,   40,  40 };
        double nominal_z    = 0.0;

        auto leg_phase = [&](int leg_id, bool tripod_a)
        {
            double phase_offset = tripod_a ? 0.0 : 0.5;
            double phase = fmod(t * step_freq + phase_offset, 1.0);

            double x = nominal_x[leg_id];
            double y = nominal_y[leg_id];

            hexapod_msgs::msg::FootTarget ft;

            if (phase < 0.5)
            {
                // STANCE: foot stays planted, moves backward relative to body motion
                double p = phase / 0.5;

                ft.x = x - step_length * (p - 0.5);
                ft.z = nominal_z;
            }
            else
            {
                // SWING: lift + move forward
                double p = (phase - 0.5) / 0.5;

                ft.x = x + step_length * (p - 0.5);
                ft.z = nominal_z + lift_height * std::sin(M_PI * p);
            }

            ft.y = y;
            return ft;
        };

        // Fill tripod A
        for (int i = 0; i < 3; i++)
            msg.foot_targets[tripodA[i]] = leg_phase(tripodA[i], true);

        // Fill tripod B
        for (int i = 0; i < 3; i++)
            msg.foot_targets[tripodB[i]] = leg_phase(tripodB[i], false);

        foot_pub_->publish(msg);
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