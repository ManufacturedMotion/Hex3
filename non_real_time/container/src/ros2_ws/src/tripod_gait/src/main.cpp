#include "tripod_gait.hpp"

int main(
    int argc,
    char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<TripodGaitNode>());

    rclcpp::shutdown();

    return 0;
}