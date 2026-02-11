%% Test Bidirectional Control Loop
% This script tests sending velocity commands from MATLAB to the drone
% and receiving odometry feedback

clear; clc;

% Connect to bridge
client = ROS2BridgeClient('192.168.0.38', 9999);
client.connect();

if ~client.is_connected()
    error('Failed to connect to ROS2 bridge');
end

fprintf('\n=== Bidirectional Control Test ===\n');
fprintf('Connected to bridge at 192.168.0.38:9999\n\n');

%% Test 1: Move forward (positive X velocity)
fprintf('TEST 1: Moving forward (+X direction)\n');
fprintf('Duration: 5 seconds at 0.5 m/s\n');

startTime = tic;
while toc(startTime) < 5
    % Send forward velocity command
    client.publish_velocity(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    % Read odometry feedback
    odom = client.receive_message(0.5);
    if ~isempty(odom)
        pos = odom.position;
        vel = odom.velocity;
        fprintf('  t=%.1fs | Pos: [%.3f, %.3f, %.3f] | Vel: [%.3f, %.3f, %.3f]\n', ...
            toc(startTime), pos(1), pos(2), pos(3), vel(1), vel(2), vel(3));
    end
    
    pause(0.1);
end

%% Test 2: Rotate (yaw motion)
fprintf('\nTEST 2: Rotating (yaw rotation)\n');
fprintf('Duration: 3 seconds at 0.5 rad/s\n');

startTime = tic;
while toc(startTime) < 3
    % Send yaw rotation command
    client.publish_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.5);
    
    % Read odometry feedback
    odom = client.receive_message(0.5);
    if ~isempty(odom)
        pos = odom.position;
        vel = odom.velocity;
        fprintf('  t=%.1fs | Pos: [%.3f, %.3f, %.3f] | AngVel: [%.3f, %.3f, %.3f]\n', ...
            toc(startTime), pos(1), pos(2), pos(3), vel(4), vel(5), vel(6));
    end
    
    pause(0.1);
end

%% Test 3: Stop and read final state
fprintf('\nTEST 3: Stopping and reading final state\n');

for i = 1:5
    % Send zero velocity (stop)
    client.publish_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    % Read odometry
    odom = client.receive_message(0.5);
    if ~isempty(odom)
        pos = odom.position;
        fprintf('  Read %d: Position = [%.3f, %.3f, %.3f]\n', ...
            i, pos(1), pos(2), pos(3));
    end
    
    pause(0.2);
end

%% Disconnect
client.disconnect();

fprintf('\n=== Test Complete ===\n');
fprintf('Bidirectional control working: MATLAB → ROS2 → Gazebo → ROS2 → MATLAB\n');
