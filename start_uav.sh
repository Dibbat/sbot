#!/bin/bash
# Spawn quadrotor in Gazebo simulator

# Setup
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

# ROS 2 Configuration
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Parse arguments
QUADROTOR_ID=${1:-1}
X_POS=${2:--2.5}
Y_POS=${3:-2.5}
Z_POS=${4:-0.5}

echo "[INFO] Spawning quadrotor_$QUADROTOR_ID..."
echo "[INFO] Position: ($X_POS, $Y_POS, $Z_POS)"
echo "[INFO] ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# Spawn quadrotor
ros2 launch imav_2017 spawn_quadrotor.py id:=${QUADROTOR_ID} x:=${X_POS} y:=${Y_POS} z:=${Z_POS}

echo "[INFO] Quadrotor spawn complete"