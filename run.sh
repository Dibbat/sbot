#!/bin/bash
# Stop simulator and clean up all processes

echo "[INFO] Stopping simulator..."

# Kill all Gazebo and ROS related processes
pkill -9 gazebo 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 spawn_entity 2>/dev/null || true
pkill -9 robot_state_publisher 2>/dev/null || true
pkill -9 fastdds 2>/dev/null || true

echo "[INFO] All processes stopped"
echo ""
echo "[USAGE]"
echo "  Start simulator:  ./run_gzb.sh"
echo "  Spawn drone:      ./start_uav.sh [id] [x] [y] [z]"
echo "  Stop simulator:   ./run.sh"
echo ""
echo "[EXAMPLES]"
echo "  ./start_uav.sh           # Spawn quadrotor_1 at default position"
echo "  ./start_uav.sh 2 1 2 0.5 # Spawn quadrotor_2 at (1, 2, 0.5)"
echo ""
echo "[VERIFY CONNECTION]"
echo "  wsl bash -c \"export ROS_DOMAIN_ID=25 && ros2 topic list\""
