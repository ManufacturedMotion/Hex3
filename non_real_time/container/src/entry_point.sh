#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

cd /app/src/ros2_ws

# Source the local workspace
source install/local_setup.bash

ros2 launch hexapod_manager hexapod.launch.py