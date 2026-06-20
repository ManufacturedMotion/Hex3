#pragma once

#include "hexapod_msgs/msg/foot_target_array.hpp"
#include "hexapod_msgs/msg/body_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define ROTATION_MAGNITUDE_SCALE 420.0
#define ROTATION_MAGNITUDE_SCALE_SQUARED (ROTATION_MAGNITUDE_SCALE * ROTATION_MAGNITUDE_SCALE)

class Pose6D
{
    public:
        Pose6D() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
        Pose6D(double x, double y, double z, double roll, double pitch, double yaw) : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
        Pose6D(const Pose6D& other) : x(other.x), y(other.y), z(other.z), roll(other.roll), pitch(other.pitch), yaw(other.yaw) {}
        // Pose6D(const Pose3D& pose3d); : x(pose3d.x), y(pose3d.y), z(pose3d.z), roll(0), pitch(0), yaw(0) {}
        Pose6D(const geometry_msgs::msg::Twist::SharedPtr msg) : x(msg->linear.x), y(msg->linear.y), z(msg->linear.z), roll(msg->angular.x), pitch(msg->angular.y), yaw(msg->angular.z) {}

        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;

        Pose6D operator+( const Pose6D& other );
        Pose6D operator-( const Pose6D& other );
        Pose6D operator*( double scalar );
        Pose6D operator/( double scalar );
        Pose6D operator*(const Pose6D& multipler);

        // Pose6D operator=(const Pose6D& other);

        void operator+=( const Pose6D& other );
        void operator-=( const Pose6D& other );
        void operator*=( double scalar );
        void operator*=( const Pose6D& multiplier );

        hexapod_msgs::msg::BodyPose toBodyPose();

        double magnitude();
        double scaledMagnitude();

        Pose6D unitVector();

};

class Pose3D
{
    public:
        Pose3D() : x(0), y(0), z(0) {}
        Pose3D(double x, double y, double z) : x(x), y(y), z(z) {}
        Pose3D(const Pose3D& other) : x(other.x), y(other.y), z(other.z) {}
        Pose3D(const Pose6D& pose6d) : x(pose6d.x), y(pose6d.y), z(pose6d.z) {}
        double x;
        double y;
        double z;

        Pose3D operator+( const Pose3D& other );
        Pose3D operator-( const Pose3D& other );
        Pose3D operator*( double scalar );

        void operator+=( const Pose3D& other );
        void operator-=( const Pose3D& other );
        void operator*=( double scalar );

        
};
