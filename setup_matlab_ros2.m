% Setup MATLAB for ROS2 communication with Docker container via TCP Bridge

% Set ROS Domain ID
setenv('ROS_DOMAIN_ID', '25');

% Set FastRTPS profile to match Docker container
% Get the path to the project directory
proj_dir = fileparts(mfilename('fullpath'));
fastrtps_profile = fullfile(proj_dir, 'DEFAULT_FASTRTPS_PROFILE.xml');
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_profile);

% Set RMW implementation
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

% Verify settings
fprintf('ROS Configuration:\n');
fprintf('  ROS_DOMAIN_ID: %s\n', getenv('ROS_DOMAIN_ID'));
fprintf('  FASTRTPS_DEFAULT_PROFILES_FILE: %s\n', getenv('FASTRTPS_DEFAULT_PROFILES_FILE'));
fprintf('  RMW_IMPLEMENTATION: %s\n', getenv('RMW_IMPLEMENTATION'));
fprintf('\n');

fprintf('========================================\n');
fprintf('ROS2/Simulink Connection Options:\n');
fprintf('========================================\n\n');

fprintf('OPTION 1: Direct ROS2 (requires Docker container network access)\n');
fprintf('  Status: Limited on Windows with Docker\n\n');

fprintf('OPTION 2: TCP Bridge (RECOMMENDED for Windows)\n');
fprintf('  Make sure the TCP bridge is running in Docker:\n');
fprintf('    docker-compose exec imav_dev bash -c "python3 ros2_tcp_bridge.py"\n\n');

% Try direct ROS2 discovery first
fprintf('Attempting direct ROS2 discovery (10 second timeout)...\n');
try
    topics_direct = ros2("topic", "list");
    if ~isempty(topics_direct) && length(topics_direct) > 2
        fprintf('✓ SUCCESS: Found %d topics via direct ROS2!\n\n', length(topics_direct));
        fprintf('Available topics:\n');
        disp(topics_direct);
    else
        fprintf('✗ Only found %d topics (expected more from Gazebo)\n', length(topics_direct));
        fprintf('  Falling back to TCP Bridge...\n\n');
        use_tcp_bridge();
    end
catch ME
    fprintf('✗ Direct ROS2 failed: %s\n', ME.message);
    fprintf('  Falling back to TCP Bridge...\n\n');
    use_tcp_bridge();
end

function use_tcp_bridge()
    % Use TCP Bridge to connect to Docker ROS2
    fprintf('Starting TCP Bridge connection...\n');
    try
        global rosbridge;
        rosbridge = ROS2BridgeClient('localhost', 9999);
        rosbridge.connect();
        fprintf('✓ Connected to TCP Bridge at localhost:9999\n');
        fprintf('  Use rosbridge to publish/subscribe to ROS2 topics\n');
    catch ME
        fprintf('✗ TCP Bridge connection failed: %s\n', ME.message);
        fprintf('\nMake sure the bridge is running:\n');
        fprintf('  docker-compose exec imav_dev bash -c "python3 ros2_tcp_bridge.py"\n');
    end
end
