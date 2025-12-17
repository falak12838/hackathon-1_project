#!/bin/bash

# Script to build the ROS2 nervous system package
# This script should be run in a ROS2 environment with colcon installed

# Navigate to the workspace
cd ~/ros2_ws/src

# Clone or copy the package to the workspace
# (assuming this package is already in the src directory)

# Build the package
colcon build --packages-select ros2_nervous_system

# Source the setup file
source install/setup.bash

# Optionally run tests
colcon test --packages-select ros2_nervous_system
colcon test-result --all

echo "Build completed. To run the node:"
echo "ros2 run ros2_nervous_system nervous_system_node"