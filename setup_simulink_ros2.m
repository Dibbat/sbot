%% Configure Simulink ROS2 Network Settings
% Run this before opening your Simulink model

% Get WSL2 IP address
[~, wsl_ip_raw] = system('wsl hostname -I');
wsl_ips = strsplit(strtrim(wsl_ip_raw));
wsl_ip = wsl_ips{1};  % Use first IP (usually 172.30.x.x)

fprintf('Detected WSL2 IP: %s\n', wsl_ip);

% Choose middleware: Fast DDS (default) or Cyclone DDS
use_cyclonedds = false;

% Set MATLAB environment for ROS2
setenv('ROS_DOMAIN_ID', '25');
if use_cyclonedds
    config_file = 'd:\IMAV_2017_Virtual_Challenge\cyclonedds_config_windows.xml';
    config_content = sprintf(['<?xml version="1.0" encoding="UTF-8"?>\n' ...
        '<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" ' ...
        'xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">\n' ...
        '  <Domain id="any">\n' ...
        '    <General>\n' ...
        '      <AllowMulticast>false</AllowMulticast>\n' ...
        '    </General>\n' ...
        '    <Discovery>\n' ...
        '      <Peers>\n' ...
        '        <Peer address="%s"/>\n' ...
        '        <Peer address="127.0.0.1"/>\n' ...
        '        <Peer address="localhost"/>\n' ...
        '      </Peers>\n' ...
        '    </Discovery>\n' ...
        '  </Domain>\n' ...
        '</CycloneDDS>'], wsl_ip);
    fid = fopen(config_file, 'w');
    fprintf(fid, '%s', config_content);
    fclose(fid);

    setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp');
    setenv('CYCLONEDDS_URI', 'file://d:/IMAV_2017_Virtual_Challenge/cyclonedds_config_windows.xml');
    setenv('ROS_DISCOVERY_SERVER', '');
else
    setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
    setenv('CYCLONEDDS_URI', '');
    setenv('ROS_DISCOVERY_SERVER', sprintf('%s:11811', wsl_ip));
end

fprintf('ROS2 Network Configuration:\n');
fprintf('  Domain ID: 25\n');
fprintf('  Middleware: %s\n', getenv('RMW_IMPLEMENTATION'));
fprintf('  WSL2 Node Host: %s\n', wsl_ip);
if use_cyclonedds
    fprintf('  CYCLONEDDS_URI: file://d:/IMAV_2017_Virtual_Challenge/cyclonedds_config_windows.xml\n');
else
    fprintf('  ROS_DISCOVERY_SERVER: %s:11811\n', wsl_ip);
end
fprintf('\nConfiguration complete!\n');
fprintf('Now open your Simulink model and set in Model Settings:\n');
fprintf('  Hardware Implementation > Hardware Board: Robot Operating System 2 (ROS 2)\n');
fprintf('  In ROS Network dialog, set Node Host IP to: %s\n', wsl_ip);
fprintf('  NOTE: Use normal Simulation mode (not Deploy to Hardware)\n');

% Optionally verify connection
fprintf('\nVerifying ROS2 connection...\n');
try
    topics = ros2("topic", "list");
    fprintf('✓ Connected! Found %d topics\n', numel(split(topics, newline)) - 1);
    
    % Check for quadrotor topics
    if contains(topics, '/quadrotor_1/')
        fprintf('✓ Quadrotor topics detected!\n');
    else
        fprintf('⚠ Quadrotor not detected. Is Gazebo running in WSL?\n');
    end
catch ME
    fprintf('✗ Connection failed: %s\n', ME.message);
    fprintf('  Make sure Gazebo is running in WSL\n');
end
