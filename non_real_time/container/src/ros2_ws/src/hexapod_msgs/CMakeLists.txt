cmake_minimum_required(VERSION 3.8)
project(hexapod_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegCommand.msg"
  "msg/BodyPose.msg"
  "msg/FootTarget.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()