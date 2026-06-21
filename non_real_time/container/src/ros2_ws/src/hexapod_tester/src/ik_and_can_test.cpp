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
        hexapod_msgs::msg::BodyPose pose;

        auto lerp = [](double a, double b, double s)
        {
            return a + (b - a) * s;
        };

        struct SweepAxis
        {
            double magnitude;
        };

        const SweepAxis axes[] =
        {
            {75.0}, // x
            {75.0}, // y
            {40.0},  // z (centered around -180)
            {0.5},   // roll
            {0.5},   // pitch
            {0.5}    // yaw
        };

        constexpr double neutral_z = -180.0;

        // Four phases per DOF:
        // 0 -> +max
        // +max -> 0
        // 0 -> -max
        // -max -> 0
        constexpr double phase_duration = 1.0;

        constexpr int phases_per_axis = 4;

        const double total_cycle =
            std::size(axes) *
            phases_per_axis *
            phase_duration;

        double cycle_time = std::fmod(t, total_cycle);

        int axis_index =
            static_cast<int>(cycle_time /
            (phases_per_axis * phase_duration));

        double axis_time =
            cycle_time -
            axis_index *
            phases_per_axis *
            phase_duration;

        int phase =
            static_cast<int>(axis_time /
            phase_duration);

        double local =
            (axis_time -
            phase * phase_duration) /
            phase_duration;

        double value = 0.0;
        double mag = axes[axis_index].magnitude;

        switch (phase)
        {
            case 0: // 0 -> +max
                value = lerp(0.0, mag, local);
                break;

            case 1: // +max -> 0
                value = lerp(mag, 0.0, local);
                break;

            case 2: // 0 -> -max
                value = lerp(0.0, -mag, local);
                break;

            case 3: // -max -> 0
                value = lerp(-mag, 0.0, local);
                break;
        }

        // Neutral pose
        pose.x = 0.0;
        pose.y = 0.0;
        pose.z = neutral_z;
        pose.roll = 0.0;
        pose.pitch = 0.0;
        pose.yaw = 0.0;

        switch (axis_index)
        {
            case 0:
                pose.x = value;
                break;

            case 1:
                pose.y = value;
                break;

            case 2:
                pose.z = neutral_z + value;
                break;

            case 3:
                pose.roll = value;
                break;

            case 4:
                pose.pitch = value;
                break;

            case 5:
                pose.yaw = value;
                break;
        }

        
        for (uint8_t i = 0; i < 6; i++) {
            hexapod_msgs::msg::BodyPose msg;
            msg.x = pose.x;
            msg.y = pose.y;
            msg.z = pose.z;
            msg.roll = pose.roll;
            msg.pitch = pose.pitch;
            msg.yaw = pose.yaw;
            msg.leg_number = i;
            body_pub_->publish(msg);
        }
    }

    void publishFootTargets(double t)
    {
        hexapod_msgs::msg::FootTargetArray msg;

        for (int i = 0; i < 6; i++) {
            hexapod_msgs::msg::FootTarget ft;
            ft.x = 0.0;
            ft.y = 0.0;
            ft.z = 0.0;
            msg.foot_targets[i] = ft;
        }
        foot_pub_->publish(msg);
        // const double step_freq   = 1.2;
        // const double step_length = 50.0;   // forward stride in mm
        // const double lift_height = 30.0;

        // // Tripod split (classic alternating diagonals)
        // const int tripodA[3] = {0, 3, 4};
        // const int tripodB[3] = {1, 2, 5};

        // // These are your FIXED nominal foot placements in BODY frame
        // // You SHOULD tune these to your real geometry
        // double nominal_x[6] = { 100,  100,  0, -100, -100,  0 };
        // double nominal_y[6] = { 120,   40, 120,  120,   40,  40 };
        // double nominal_z    = 0.0;

        // auto leg_phase = [&](int leg_id, bool tripod_a)
        // {
        //     double phase_offset = tripod_a ? 0.0 : 0.5;
        //     double phase = fmod(t * step_freq + phase_offset, 1.0);

        //     double x = nominal_x[leg_id];
        //     double y = nominal_y[leg_id];

        //     hexapod_msgs::msg::FootTarget ft;

        //     if (phase < 0.5)
        //     {
        //         // STANCE: foot stays planted, moves backward relative to body motion
        //         double p = phase / 0.5;

        //         ft.x = x - step_length * (p - 0.5);
        //         ft.z = nominal_z;
        //     }
        //     else
        //     {
        //         // SWING: lift + move forward
        //         double p = (phase - 0.5) / 0.5;

        //         ft.x = x + step_length * (p - 0.5);
        //         ft.z = nominal_z + lift_height * std::sin(M_PI * p);
        //     }

        //     ft.y = y;
        //     return ft;
        // };

        // // Fill tripod A
        // for (int i = 0; i < 3; i++)
        //     msg.foot_targets[tripodA[i]] = leg_phase(tripodA[i], true);

        // // Fill tripod B
        // for (int i = 0; i < 3; i++)
        //     msg.foot_targets[tripodB[i]] = leg_phase(tripodB[i], false);

        // foot_pub_->publish(msg);
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