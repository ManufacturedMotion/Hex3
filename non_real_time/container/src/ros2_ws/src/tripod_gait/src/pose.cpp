#include "pose.hpp"
#include <cmath>

Pose6D Pose6D::operator+( const Pose6D& other ) {
    return Pose6D (
        x + other.x,
        y + other.y,
        z + other.z,
        roll + other.roll,
        pitch + other.pitch,
        yaw + other.yaw
    );
}

Pose6D Pose6D::operator-( const Pose6D& other ) {
    return Pose6D(
        x - other.x,
        y - other.y,
        z - other.z,
        roll - other.roll,
        pitch - other.pitch,
        yaw - other.yaw
    );
}

Pose6D Pose6D::operator*( double scalar ) {
    return Pose6D(
        x * scalar,
        y * scalar,
        z * scalar,
        roll * scalar,
        pitch * scalar,
        yaw * scalar
    );
}

Pose6D Pose6D::operator/( double scalar ) {
    return Pose6D (
        x / scalar,
        y / scalar,
        z / scalar,
        roll / scalar,
        pitch / scalar,
        yaw / scalar
    );
}

void Pose6D::operator*=(const Pose6D& multiplier) {
    x *= multiplier.x;
    y *= multiplier.y;
    z *= multiplier.z;
    roll *= multiplier.roll;
    pitch *= multiplier.pitch;
    yaw *= multiplier.yaw;
}

Pose6D Pose6D::operator*(const Pose6D& multiplier) {
    return Pose6D(
        x * multiplier.x,
        y * multiplier.y,
        z * multiplier.z,
        roll * multiplier.roll,
        pitch * multiplier.pitch,
        yaw * multiplier.yaw
    )
}

// Pose6D Pose6D::operator=( const Pose6D& other ) {
//     return Pose6D(other);
// }

Pose6D Pose6D::unitVector() {
    return Pose6D(*(this) * (1.0 / magnitude()));
}

double Pose6D::magnitude() {
    return std::sqrt(x*x + y*y + z*z + roll*roll + pitch*pitch + yaw*yaw);
}


void Pose6D::operator+=( const Pose6D& other ) {
    x += other.x;
    y += other.y;
    z += other.z;
    roll += other.roll;
    pitch += other.pitch;
    yaw += other.yaw;
}

void Pose6D::operator-=( const Pose6D& other ) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    roll -= other.roll;
    pitch -= other.pitch;
    yaw -= other.yaw;
}

void Pose6D::operator*=( double scalar ) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    roll *= scalar;
    pitch *= scalar;
    yaw *= scalar;
}

hexapod_msgs::msg::BodyPose Pose6D::toBodyPose() {
    hexapod_msgs::msg::BodyPose body_pose;
    body_pose.x = x;
    body_pose.y = y;
    body_pose.z = z;
    body_pose.roll = roll;
    body_pose.pitch = pitch;
    body_pose.yaw = yaw;
    return body_pose;
}

void Pose3D::operator+=( const Pose3D& other ) {
    x += other.x;
    y += other.y;
    z += other.z;
}

void Pose3D::operator-=( const Pose3D& other ) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
}

void Pose3D::operator*=( double scalar ) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
}

