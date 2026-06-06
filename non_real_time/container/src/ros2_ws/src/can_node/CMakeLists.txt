cmake_minimum_required(VERSION 3.16)
project(ros2_can_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(jazzy_can_node src/jazzy_can_node.cpp)
ament_target_dependencies(jazzy_can_node rclcpp std_msgs)

install(TARGETS jazzy_can_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
