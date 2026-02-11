#!/bin/bash
# Run ROS2 Docker container on Windows/WSL
# Note: GPU acceleration not available on Windows Docker
# This script is for WSL 2 with standard Docker support
#
# REQUIREMENTS:
#   - WSL 2 with Ubuntu
#   - XLaunch running on Windows (display 0)
#   - Docker Desktop with WSL 2 backend

# Enable X11 access for WSL
xhost +local:root 2>/dev/null || true

# Container name
CONTAINER_NAME="imav_ros2_dev"

# Check if container is already running
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[INFO] Container $CONTAINER_NAME already exists. Removing it..."
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
fi

# Run Docker container with full development environment
# For Windows/WSL: Use host.docker.internal:0.0 as DISPLAY
# GPU support (--runtime=nvidia) requires Linux with NVIDIA drivers
# This version works on Windows without GPU acceleration
echo "[INFO] Starting ROS2 Docker container for IMAV development..."
echo "[INFO] Make sure XLaunch is running on Windows (Display 0)"
echo ""

docker run -it \
    --name "$CONTAINER_NAME" \
    --env="DISPLAY=host.docker.internal:0.0" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=25" \
    --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
    --volume="$PWD:/app:rw" \
    --workdir="/app" \
    --privileged \
    --network="host" \
    imav_2017:latest \
    bash

echo "[INFO] Container stopped."
