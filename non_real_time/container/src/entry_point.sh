#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

cd /app/src/ros2_ws

# Rebuild only the packages needed for the hexapod launch when the source tree is mounted in.
# This avoids rebuilding the full workspace while still ensuring the required nodes are installed.
colcon build --packages-up-to hexapod_manager inverse_kinematics can control tripod_gait

# Source the local workspace
source install/local_setup.bash

ros2 launch hexapod_manager hexapod.launch.py