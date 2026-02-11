#!/bin/bash

# Source ROS 2 base environment
source /opt/ros/humble/setup.bash

# Source workspace built packages
source /app/install/setup.bash

# Execute user command or bash
exec "$@"
