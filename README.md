== IMAV2017 Virtual Challenge ==

Simulation environment using ROS and Gazebo for the IMAV2017 Virtual Challenge. It has been succesfully tested on Ubuntu 16.04 (Xenial) 64bit.

Please read the PDF file IMAV2017_Virtual_Challenge_Guide.pdf for detailled instructions and installation notes.

Quick install and run process (using bash, if you use a different shell you may need to adapt some scripts):
  - if not done make scripts executable:
    chmod +x setup.sh
    chmod +x run_gzb.sh
    chmod +x start_uav.sh

  - run setup script (this should be done once for a fresh install without ROS or gazebo alreaded installed, in case of error run required commands by hand step by step)
    ./setup.sh

  - start gazebo environment simulator and ROS server
    ./run_gzb.sh

  - add a robot to the scene with default ID
    ./start_uav.sh

Once ROS/gazebo simulation is running, you can use Simulink or your own system to control the UAV

## MATLAB/Simulink ROS2 Connection

### Available ROS2 Topics (quadrotor_1):

**Control Input:**
- `/quadrotor_1/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Sensor Outputs:**
- `/quadrotor_1/imu` (sensor_msgs/Imu) - IMU data (100 Hz)
- `/quadrotor_1/scan` (sensor_msgs/LaserScan) - Lidar data (40 Hz, 1081 rays)
- `/quadrotor_1/sonar_height` (sensor_msgs/Range) - Sonar height (10 Hz)
- `/quadrotor_1/front/image_raw` (sensor_msgs/Image) - Front camera (640x360, 60 Hz)
- `/quadrotor_1/front/camera_info` (sensor_msgs/CameraInfo) - Front camera info
- `/quadrotor_1/bottom/image_raw` (sensor_msgs/Image) - Bottom camera (640x360, 60 Hz)
- `/quadrotor_1/bottom/camera_info` (sensor_msgs/CameraInfo) - Bottom camera info

### MATLAB Setup:

**IMPORTANT: Network Configuration**

MATLAB on Windows needs to communicate with ROS2 in WSL. Check your network setup:

```bash
# In WSL, check your IP
hostname -I
```

If MATLAB and WSL are on different networks, you need to configure WSL to use bridge mode or use the Windows host IP.

**For WSL2 (most common):**
- WSL2 uses a virtual network adapter
- MATLAB should connect to localhost (127.0.0.1)
- Make sure ROS_DOMAIN_ID is the same (0) in both MATLAB and WSL

**MATLAB Connection Code:**

```matlab
% Initialize ROS2 in MATLAB (R2021a or later)
setenv("ROS_DOMAIN_ID", "0")

% Create ROS2 node
node = ros2node("/matlab_controller");

% Create publisher for velocity commands
cmd_pub = ros2publisher(node, "/quadrotor_1/cmd_vel", "geometry_msgs/Twist");

% Create subscribers for sensors
imu_sub = ros2subscriber(node, "/quadrotor_1/imu");
scan_sub = ros2subscriber(node, "/quadrotor_1/scan");
cam_sub = ros2subscriber(node, "/quadrotor_1/front/image_raw");

% Send velocity command
msg = ros2message(cmd_pub);
msg.linear.z = 1.0;  % Ascend at 1 m/s
send(cmd_pub, msg);

% Read sensor data
imu_data = receive(imu_sub, 3);
scan_data = receive(scan_sub, 3);
```

### Simulink Setup:

1. Open Simulink and create a new model
2. Add ROS 2 blocks from Simulink Library Browser:
   - **ROS 2 Publish** block for `/quadrotor_1/cmd_vel` (geometry_msgs/Twist)
   - **ROS 2 Subscribe** blocks for sensor topics
3. Configure each block:
   - Set Domain ID: 0
   - Set Topic name (e.g., `/quadrotor_1/cmd_vel`)
   - Set Message type (e.g., `geometry_msgs/Twist`)
4. Connect control logic to publish block
5. Run simulation

### Test from Command Line:

```bash
# Ascend
ros2 topic pub /quadrotor_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Descend
ros2 topic pub /quadrotor_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Rotate
ros2 topic pub /quadrotor_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" --once

# List all topics
ros2 topic list

# Echo IMU data
ros2 topic echo /quadrotor_1/imu
```

