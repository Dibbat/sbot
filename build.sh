#!/bin/bash
# Build script for IMAV 2017 ROS2 project
source /opt/ros/humble/setup.bash
cd ~/colcon_ws
colcon build --symlink-install "$@"
