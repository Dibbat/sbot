#!/bin/bash
# Monitor ROS2 topics in domain 25 continuously

echo "=========================================="
echo "ROS2 Topic Monitor - Domain 25"
echo "=========================================="
echo ""

source /opt/ros/humble/setup.bash

# Repeat every 3 seconds
while true; do
    clear
    echo "=========================================="
    echo "ROS2 Topic Monitor - Domain 25"
    echo "Updated: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=========================================="
    echo ""
    
    echo "[AVAILABLE TOPICS]"
    ros2 topic list
    
    echo ""
    echo "[TOPIC TYPES]"
    ros2 topic list -t
    
    echo ""
    echo "[ACTIVE PUBLISHERS & SUBSCRIBERS]"
    ros2 node list
    
    echo ""
    echo "Press Ctrl+C to stop monitoring..."
    sleep 3
done
