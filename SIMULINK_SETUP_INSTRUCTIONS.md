# Simulink ROS2 Configuration Guide

## The error you're seeing means Simulink is trying to connect via SSH (for external hardware deployment). We're using ROS2 network communication instead.

## Step-by-Step Configuration:

### 1. First, Run in MATLAB:
```matlab
cd('d:\IMAV_2017_Virtual_Challenge')
setup_simulink_ros2
```
This configures the MATLAB environment for ROS2.

---

### 2. Open Your Simulink Model

---

### 3. Configure Model Settings:

**Option A: For Simulation Mode (RECOMMENDED)**

1. Open your Simulink model
2. **Configuration Parameters** (Ctrl+E) → **Solver**
   - Set Solver to: **Fixed-step**
   - Fixed-step size: **0.01** (or appropriate for your control loop)

3. Add ROS2 blocks directly to your model:
   - **ROS2 Subscribe Block:**
     - Block Path: `Simulink > ROS Toolbox > ROS 2 Subscribe`
     - Parameters:
       - Topic source: **Select from ROS network** 
       - Click **Select** → Choose `/quadrotor_1/odom`
       - Message type: `nav_msgs/Odometry`
   
   - **ROS2 Publish Block:**
     - Block Path: `Simulink > ROS Toolbox > ROS 2 Publish`
     - Parameters:
       - Topic name: `/quadrotor_1/cmd_vel`
       - Message type: `geometry_msgs/Twist`

4. Click **Simulation > Run** (or press Ctrl+T)
   - Simulink will create ROS2 nodes automatically during simulation
   - No SSH connection needed!

---

**Option B: For Code Generation/Deployment (Advanced)**

Only use this if you're generating code to run outside MATLAB:

1. **Model Settings** (Ctrl+E) → **Code Generation**
   - System target file: **ert.tlc**
   
2. **Hardware Implementation**
   - Hardware board: **Robot Operating System 2 (ROS 2)**
   
3. **Click "Configure ROS Network Addresses"**
   - Node Host (ROS2): **Custom** → Enter: `172.30.192.180` (WSL2 IP)
   - Domain ID: `0`
   - RMW Implementation: `rmw_cyclonedds_cpp`

---

## Current System Status:

✅ **WSL2 (Ubuntu 22.04):**
- Gazebo running with FastDDS
- ROS_DOMAIN_ID=0
- IP: 172.30.192.180
- Topics: `/quadrotor_1/*` available

✅ **Windows (MATLAB):**
- CycloneDDS configured with peer discovery
- ROS_DOMAIN_ID=0
- Config: cyclonedds_config_windows.xml

---

## Available Topics:

```
/quadrotor_1/cmd_vel              (geometry_msgs/Twist) - Control input
/quadrotor_1/odom                 (nav_msgs/Odometry) - Position/velocity
/quadrotor_1/imu                  (sensor_msgs/Imu) - IMU data
/quadrotor_1/scan                 (sensor_msgs/LaserScan) - Lidar
/quadrotor_1/front/image_raw      (sensor_msgs/Image) - Front camera
/quadrotor_1/bottom/image_raw     (sensor_msgs/Image) - Bottom camera
```

---

## Troubleshooting:

### If ROS2 Subscribe shows "No topics available":
1. Verify MATLAB environment:
   ```matlab
   getenv('ROS_DOMAIN_ID')        % Should be '0'
   getenv('RMW_IMPLEMENTATION')   % Should be 'rmw_cyclonedds_cpp'
   ros2 topic list                % Should show topics
   ```

2. If no topics visible, restart MATLAB and run `setup_simulink_ros2` again

### If Simulink asks for SSH credentials:
- You selected "Deploy to Hardware" mode
- Change to **Normal Simulation Mode** instead
- Or remove Hardware Implementation settings

---

## Quick Test in MATLAB:

```matlab
% After running setup_simulink_ros2

% Check topics
topics = ros2("topic", "list")

% Test subscribe
sub = ros2subscriber("/quadrotor_1/odom");
pause(1);
msg = sub.LatestMessage

% Test publish
pub = ros2publisher("/quadrotor_1/cmd_vel", "geometry_msgs/Twist");
cmd = ros2message(pub);
cmd.linear.x = 0.5;  % Forward
send(pub, cmd);
```

---

## Important Notes:

- **For normal Simulink simulation**: You DON'T need SSH or hardware deployment
- **ROS2 blocks work directly in simulation mode** - just add Subscribe/Publish blocks
- The blocks will automatically connect to ROS2 network when you run the simulation
- SSH deployment is only for generating standalone C++ code to run on external hardware
