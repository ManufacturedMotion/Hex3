#include "inverse_kinematics.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<InverseKinematicsNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}