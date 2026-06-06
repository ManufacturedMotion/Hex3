cmake_minimum_required(VERSION 3.8)
project(can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(hexapod_msgs REQUIRED)

add_executable(can_interface src/can_interface.cpp)
target_include_directories(can_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(can_interface PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  can_interface
  rclcpp
  std_msgs
  nlohmann_json
  hexapod_msgs
)

# rosidl_get_typesupport_target(
#   cpp_typesupport_target
#   hexapod_msgs
#   rosidl_typesupport_cpp
# )

target_link_libraries(can_interface
  "${cpp_typesupport_target}"
)

add_executable(leg_command_publisher src/leg_command_publisher.cpp)

target_include_directories(leg_command_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(leg_command_publisher PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  leg_command_publisher
  rclcpp
  std_msgs
  hexapod_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target_pub
  hexapod_msgs
  rosidl_typesupport_cpp
)

target_link_libraries(leg_command_publisher
  "${cpp_typesupport_target_pub}"
)

install(TARGETS
  can_interface
  leg_command_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES resource/leg_groups.json
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()