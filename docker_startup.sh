#!/bin/bash
# Complete IMAV 2017 Startup Script - Runs all services in Docker container
# This script launches Gazebo, spawns the drone, and starts the TCP bridge

set -e

echo "=================================================="
echo "IMAV 2017 Virtual Challenge - Complete Startup"
echo "=================================================="

# Source ROS2 and workspace
echo "[1/4] Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash
source /app/install/setup.bash

# Set environment variables
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GAZEBO_MODEL_PATH=/app/install/imav_2017/share/imav_2017/models:/usr/share/gazebo-11/models
export GAZEBO_PLUGIN_PATH=/app/install/imav_2017/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export FASTRTPS_DEFAULT_PROFILES_FILE=/app/DEFAULT_FASTRTPS_PROFILE.xml

echo "[✓] ROS2 environment ready"
echo "    ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "    GAZEBO_MODEL_PATH configured"

# Start Gazebo in background
echo ""
echo "[2/4] Starting Gazebo simulator..."
ros2 launch imav_2017 imav_indoor.launch.py &
GAZEBO_PID=$!
sleep 5

echo "[✓] Gazebo started (PID: $GAZEBO_PID)"

# Spawn the quadrotor
echo ""
echo "[3/4] Spawning quadrotor_1 in Gazebo..."
sleep 2
ros2 launch imav_2017 spawn_quadrotor.launch.py &
SPAWN_PID=$!
sleep 3

echo "[✓] Quadrotor spawned (PID: $SPAWN_PID)"

# Verify ROS2 topics
echo ""
echo "[4/4] Verifying ROS2 topics..."
ros2 topic list | head -10
echo ""

# Start TCP bridge
echo "=================================================="
echo "Starting TCP Bridge Server..."
echo "=================================================="
echo ""
python3 /app/ros2_tcp_bridge.py

# Keep running
wait
