FROM osrf/ros:humble-desktop

# Set working directory
WORKDIR /app

# Install additional ROS 2 packages needed for the IMAV challenge
RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-dev \
    ros-humble-ros2launch \
    && rm -rf /var/lib/apt/lists/*

# Copy entire workspace and reorganize into src structure
RUN mkdir -p /app/src/imav_2017

COPY package.xml CMakeLists.txt setup.py /app/src/imav_2017/
COPY launch /app/src/imav_2017/launch
COPY scripts /app/src/imav_2017/scripts
COPY urdf /app/src/imav_2017/urdf
COPY worlds /app/src/imav_2017/worlds
COPY meshes /app/src/imav_2017/meshes
COPY plugins /app/src/imav_2017/plugins
COPY resource* /app/src/imav_2017/

# Build the workspace (base image has all required packages pre-installed)
RUN bash -c "source /opt/ros/humble/setup.bash && \
    cd /app && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
