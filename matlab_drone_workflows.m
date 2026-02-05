%% ROS 2 Drone Control Workflows for quadrotor_1
% Updated for IMAV 2017 Virtual Challenge

%% Setup (Run this first)
setenv('ROS_DOMAIN_ID', '25')
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

% Verify connection
fprintf('Available topics:\n')
ros2 topic list

%% Workflow 1: Subscribe to Drone Odometry (State)
% Get position and velocity from the drone

setenv('ROS_DOMAIN_ID', '25')

% Create subscriber for odometry
odomSub = ros2subscriber('/quadrotor_1/odom', 'nav_msgs/Odometry');

% Receive one message
fprintf('Waiting for odometry data...\n')
odomData = receive(odomSub, 10); % 10 second timeout

% Display position
fprintf('\nDrone Position:\n')
fprintf('  X: %.3f m\n', odomData.pose.pose.position.x)
fprintf('  Y: %.3f m\n', odomData.pose.pose.position.y)
fprintf('  Z: %.3f m\n', odomData.pose.pose.position.z)

% Display velocity
fprintf('\nDrone Velocity:\n')
fprintf('  Linear X: %.3f m/s\n', odomData.twist.twist.linear.x)
fprintf('  Linear Y: %.3f m/s\n', odomData.twist.twist.linear.y)
fprintf('  Linear Z: %.3f m/s\n', odomData.twist.twist.linear.z)

%% Workflow 2: Publish Velocity Commands to Drone
% Manual control - send velocity commands to quadrotor_1

setenv('ROS_DOMAIN_ID', '25')

% Create publisher for velocity commands
velPub = ros2publisher('/quadrotor_1/cmd_vel', 'geometry_msgs/Twist');

% Create message
velMsg = ros2message(velPub);

% Example 1: Move forward
fprintf('Moving forward...\n')
velMsg.linear.x = 0.5;   % 0.5 m/s forward
velMsg.linear.y = 0.0;   % no lateral movement
velMsg.linear.z = 0.0;   % no vertical movement
velMsg.angular.x = 0.0;  % no roll
velMsg.angular.y = 0.0;  % no pitch
velMsg.angular.z = 0.0;  % no yaw rotation
send(velPub, velMsg)
pause(2)

% Example 2: Rotate
fprintf('Rotating...\n')
velMsg.linear.x = 0.0;
velMsg.angular.z = 0.5;  % 0.5 rad/s yaw rotation
send(velPub, velMsg)
pause(2)

% Example 3: Ascend
fprintf('Ascending...\n')
velMsg.linear.x = 0.0;
velMsg.linear.z = 0.3;   % 0.3 m/s upward
velMsg.angular.z = 0.0;
send(velPub, velMsg)
pause(2)

% Stop drone
fprintf('Stopping...\n')
velMsg.linear.x = 0.0;
velMsg.linear.y = 0.0;
velMsg.linear.z = 0.0;
velMsg.angular.x = 0.0;
velMsg.angular.y = 0.0;
velMsg.angular.z = 0.0;
send(velPub, velMsg)

%% Workflow 3: Continuous Monitoring with Real-time Visualization
% Monitor drone state and send commands in real-time

setenv('ROS_DOMAIN_ID', '25')

% Create subscribers
odomSub = ros2subscriber('/quadrotor_1/odom', 'nav_msgs/Odometry');
laserSub = ros2subscriber('/quadrotor_1/scan', 'sensor_msgs/LaserScan');

% Create publisher for commands
velPub = ros2publisher('/quadrotor_1/cmd_vel', 'geometry_msgs/Twist');

% Create figure for visualization
fig = figure('Name', 'Quadrotor_1 Monitor', 'NumberTitle', 'off');
tiledlayout(2, 2);

% Position history
posHistory.x = [];
posHistory.y = [];
posHistory.z = [];

% Loop for 60 seconds
tic
loopTime = 0;
while toc < 60 && isvalid(fig)
    loopTime = toc;
    
    % Get odometry data
    odomData = odomSub.LatestMessage;
    if ~isempty(odomData)
        % Store position
        posHistory.x = [posHistory.x, odomData.pose.pose.position.x];
        posHistory.y = [posHistory.y, odomData.pose.pose.position.y];
        posHistory.z = [posHistory.z, odomData.pose.pose.position.z];
        
        % Plot 1: 3D Position Trajectory
        nexttile(1)
        plot3(posHistory.x, posHistory.y, posHistory.z, 'b-', 'LineWidth', 2)
        hold on
        plot3(posHistory.x(end), posHistory.y(end), posHistory.z(end), 'ro', 'MarkerSize', 8)
        hold off
        xlabel('X (m)')
        ylabel('Y (m)')
        zlabel('Z (m)')
        title('3D Position Trajectory')
        grid on
        
        % Plot 2: Altitude vs Time
        nexttile(2)
        plot(loopTime, odomData.pose.pose.position.z, 'b.', 'MarkerSize', 8)
        hold on
        xlabel('Time (s)')
        ylabel('Altitude Z (m)')
        title('Altitude Over Time')
        grid on
        
        % Plot 3: XY Position
        nexttile(3)
        plot(posHistory.x, posHistory.y, 'b-', 'LineWidth', 2)
        hold on
        plot(posHistory.x(end), posHistory.y(end), 'ro', 'MarkerSize', 8)
        hold off
        xlabel('X (m)')
        ylabel('Y (m)')
        title('XY Position')
        grid on
        axis equal
        
        % Plot 4: Velocity
        nexttile(4)
        velMag = sqrt(odomData.twist.twist.linear.x^2 + ...
                      odomData.twist.twist.linear.y^2 + ...
                      odomData.twist.twist.linear.z^2);
        plot(loopTime, velMag, 'g.', 'MarkerSize', 8)
        hold on
        xlabel('Time (s)')
        ylabel('Speed (m/s)')
        title('Velocity Magnitude')
        grid on
    end
    
    % Get laser data
    scanData = laserSub.LatestMessage;
    if ~isempty(scanData) && numel(scanData.ranges) > 0
        % Find minimum distance (obstacle detection)
        validRanges = scanData.ranges(scanData.ranges > scanData.range_min & ...
                                      scanData.ranges < scanData.range_max);
        if ~isempty(validRanges)
            minDistance = min(validRanges);
            % You could use this for obstacle avoidance
        end
    end
    
    drawnow
    pause(0.1)
end

fprintf('Monitoring completed after %.1f seconds\n', toc)

%% Workflow 4: Manual Keyboard Control (Optional)
% Interactive control with arrow keys and other commands

setenv('ROS_DOMAIN_ID', '25')

% Create publisher
velPub = ros2publisher('/quadrotor_1/cmd_vel', 'geometry_msgs/Twist');
odomSub = ros2subscriber('/quadrotor_1/odom', 'nav_msgs/Odometry');

% Control parameters
linearSpeed = 0.5;   % m/s
angularSpeed = 0.5;  % rad/s
verticalSpeed = 0.3; % m/s

fprintf('Keyboard Control:\n')
fprintf('  W/S: Forward/Backward\n')
fprintf('  A/D: Rotate Left/Right\n')
fprintf('  Up/Down: Ascend/Descend\n')
fprintf('  SPACE: Stop\n')
fprintf('  Q: Quit\n\n')

% Create control figure
fig = figure('Name', 'Keyboard Control', 'NumberTitle', 'off');
set(fig, 'KeyPressFcn', @keyPress)

% Store state
controlState = struct('wPressed', false, 'sPressed', false, ...
                      'aPressed', false, 'dPressed', false, ...
                      'upPressed', false, 'downPressed', false, ...
                      'shouldQuit', false);

% Control loop
while ~controlState.shouldQuit && isvalid(fig)
    velMsg = ros2message(velPub);
    
    % Linear velocity
    velMsg.linear.x = (controlState.wPressed - controlState.sPressed) * linearSpeed;
    velMsg.linear.z = (controlState.upPressed - controlState.downPressed) * verticalSpeed;
    
    % Angular velocity
    velMsg.angular.z = (controlState.dPressed - controlState.aPressed) * angularSpeed;
    
    send(velPub, velMsg)
    
    % Display current state
    odomData = odomSub.LatestMessage;
    if ~isempty(odomData)
        fprintf('\rPos: [%.2f, %.2f, %.2f] | Vel: [%.2f, %.2f, %.2f]', ...
            odomData.pose.pose.position.x, ...
            odomData.pose.pose.position.y, ...
            odomData.pose.pose.position.z, ...
            odomData.twist.twist.linear.x, ...
            odomData.twist.twist.linear.y, ...
            odomData.twist.twist.linear.z)
    end
    
    pause(0.05)
end

% Stop drone
velMsg = ros2message(velPub);
velMsg.linear.x = 0;
velMsg.linear.y = 0;
velMsg.linear.z = 0;
send(velPub, velMsg)
fprintf('\n\nDrone stopped and control released.\n')

% Nested function for keyboard input
function keyPress(~, event)
    global controlState
    switch event.Key
        case 'w'
            controlState.wPressed = true;
        case 's'
            controlState.sPressed = true;
        case 'a'
            controlState.aPressed = true;
        case 'd'
            controlState.dPressed = true;
        case 'uparrow'
            controlState.upPressed = true;
        case 'downarrow'
            controlState.downPressed = true;
        case 'space'
            % Stop all motion
            controlState.wPressed = false;
            controlState.sPressed = false;
            controlState.aPressed = false;
            controlState.dPressed = false;
            controlState.upPressed = false;
            controlState.downPressed = false;
        case 'q'
            controlState.shouldQuit = true;
    end
end
