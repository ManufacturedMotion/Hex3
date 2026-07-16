#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

cd /app/src/ros2_ws

# Build the full workspace so the latest nodes are installed before launch.
colcon build

# Source the local workspace
source install/local_setup.bash

ros2 launch hexapod_manager hexapod.launch.py