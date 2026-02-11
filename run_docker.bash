#!/bin/bash
# Run ROS2 Docker container with GUI support on Windows/WSL
# This script sets up display sharing for GUI applications like Gazebo
# 
# REQUIREMENTS:
#   - WSL 2 with Ubuntu
#   - XLaunch running on Windows (display 0)
#   - Docker Desktop with WSL 2 backend

# Enable X11 access for WSL
xhost +local:root 2>/dev/null || true

# Container name
CONTAINER_NAME="imav_ros2_gui"

# Check if container is already running
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[INFO] Container $CONTAINER_NAME already exists. Removing it..."
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
fi

# Run Docker container with GUI support
# For Windows/WSL: Use host.docker.internal:0.0 as DISPLAY
# --env="QT_X11_NO_MITSHM=1": Fix Qt GUI rendering issues
# --volume="$PWD:/app": Mount current directory to /app in container
# --privileged: Grant additional permissions for Gazebo simulation
echo "[INFO] Starting ROS2 Docker container with GUI support..."
echo "[INFO] Make sure XLaunch is running on Windows (Display 0)"
echo ""

docker run -it \
    --name "$CONTAINER_NAME" \
    --env="DISPLAY=host.docker.internal:0.0" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="$PWD:/app:rw" \
    --workdir="/app" \
    --privileged \
    --network="host" \
    imav_2017:latest \
    bash

echo "[INFO] Container stopped."
