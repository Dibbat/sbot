#!/bin/bash
# Start Gazebo simulator with ROS 2 support

# Setup
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

# ROS 2 Configuration
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[INFO] Starting Gazebo simulator..."
echo "[INFO] ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "[INFO] RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# Kill any existing processes
echo "[INFO] Cleaning up old processes..."
pkill -9 gazebo gzserver gzclient 2>/dev/null || true
sleep 1

# Generate world file
echo "[INFO] Generating world..."
xacro install/imav_2017/share/imav_2017/worlds/xacro/imav_indoor.world.xacro -o /tmp/imav_sim.world

# Start Gazebo server and client (GUI)
echo "[INFO] Launching Gazebo server..."
gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so /tmp/imav_sim.world &
GZ_PID=$!
echo "[INFO] Gazebo server PID: $GZ_PID"

sleep 3

# Start Gazebo client (GUI)
echo "[INFO] Starting Gazebo GUI client..."
gzclient &
GZCLIENT_PID=$!
echo "[INFO] Gazebo client PID: $GZCLIENT_PID"

echo "[INFO] Gazebo started successfully"
echo "[INFO] Run './start_uav.sh' to spawn the quadrotor"

# Keep script running
wait $GZ_PID
