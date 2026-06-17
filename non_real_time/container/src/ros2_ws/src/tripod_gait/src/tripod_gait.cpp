#include "tripod_gait.hpp"

#include <cmath>

TripodGaitNode::TripodGaitNode()
    : Node("tripod_gait"),
      gait_phase_(0.0),
      cycle_time_(1.0),
      step_height_(30.0),
      max_step_length_(80.0),
      body_height_(-120.0)
{
    cmd_vel_sub_ =
        create_subscription<
            geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &TripodGaitNode::cmdVelCallback,
                this,
                std::placeholders::_1));

    foot_target_pub_ =
        create_publisher<
            hexapod_msgs::msg::FootTargetArray>(
            "/foot_targets",
            10);

    body_pose_pub_ =
        create_publisher<
            hexapod_msgs::msg::BodyPose>(
            "/body_pose",
            10);

    timer_ =
        create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(
                &TripodGaitNode::timerCallback,
                this));

    last_update_ = now();

    home_x_ =
    {
         120.0,
           0.0,
        -120.0,
         120.0,
           0.0,
        -120.0
    };

    home_y_ =
    {
         100.0,
         100.0,
         100.0,
        -100.0,
        -100.0,
        -100.0
    };
}

void TripodGaitNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_ = *msg;
}

void TripodGaitNode::timerCallback()
{
    auto current_time = now();

    double dt =
        (current_time - last_update_).seconds();

    last_update_ = current_time;

    updateGait(dt);
}

bool TripodGaitNode::legInGroupA(
    size_t leg) const
{
    return
        leg == 0 ||
        leg == 3 ||
        leg == 4;
}

void TripodGaitNode::updateGait(
    double dt)
{
    gait_phase_ += dt / cycle_time_;

    while (gait_phase_ > 1.0)
    {
        gait_phase_ -= 1.0;
    }

    double speed =
        std::hypot(
            cmd_vel_.linear.x,
            cmd_vel_.linear.y);

    speed = std::min(speed, 1.0);

    double step_length =
        speed * max_step_length_;

    hexapod_msgs::msg::FootTargetArray targets;

    for (size_t leg = 0; leg < 6; leg++)
    {
        bool group_a =
            legInGroupA(leg);

        double phase =
            group_a ?
            gait_phase_ :
            std::fmod(
                gait_phase_ + 0.5,
                1.0);

        bool swing =
            phase < 0.5;

        double local_phase =
            swing ?
            phase / 0.5 :
            (phase - 0.5) / 0.5;

        double foot_x;
        double foot_z;

        if (swing)
        {
            foot_x =
                -step_length * 0.5 +
                step_length * local_phase;

            foot_z =
                step_height_ *
                std::sin(
                    M_PI *
                    local_phase);
        }
        else
        {
            foot_x =
                 step_length * 0.5 -
                 step_length * local_phase;

            foot_z = 0.0;
        }

        double vx =
            cmd_vel_.linear.x;

        double vy =
            cmd_vel_.linear.y;

        double yaw =
            cmd_vel_.angular.z;

        double x =
            home_x_[leg] +
            foot_x * vx;

        double y =
            home_y_[leg] +
            foot_x * vy;

        y += yaw * home_x_[leg];
        x -= yaw * home_y_[leg];

        hexapod_msgs::msg::FootTarget target;

        target.leg_id = leg;

        target.x = x;
        target.y = y;
        target.z = body_height_ + foot_z;

        targets.targets.push_back(target);
    }

    foot_target_pub_->publish(targets);

    hexapod_msgs::msg::BodyPose body_pose;

    body_pose.x = 0.0;
    body_pose.y = 0.0;
    body_pose.z = 0.0;

    body_pose.roll = 0.0;
    body_pose.pitch = 0.0;
    body_pose.yaw = 0.0;

    body_pose_pub_->publish(body_pose);
}