classdef ROS2BridgeClient < handle
    % ROS2BridgeClient TCP client for ROS2 Bridge communication
    % Connects to ros2_tcp_bridge.py running in Docker
    % 
    % Usage:
    %   client = ROS2BridgeClient('192.168.0.38', 9999);
    %   client.connect();
    %   odom = client.receive_message(5);
    %   disp(odom);
    
    properties (Access = private)
        tcpClient       % tcpclient object
        host            % Server host (IP)
        port            % Server port
        isConnected     % Connection status flag
    end
    
    methods
        function obj = ROS2BridgeClient(host, port)
            % Constructor: Initialize TCP bridge client
            % Usage: client = ROS2BridgeClient('192.168.0.38', 9999);
            
            if nargin < 1, host = '192.168.0.38'; end  % Default to Ubuntu VM IP
            if nargin < 2, port = 9999; end
            
            obj.host = host;
            obj.port = port;
            obj.tcpClient = [];
            obj.isConnected = false;
        end
        
        function connect(obj)
            % Connect to TCP bridge server
            try
                obj.tcpClient = tcpclient(obj.host, obj.port, ...
                    'ConnectTimeout', 5, ...
                    'Timeout', 5);
                obj.isConnected = true;
                fprintf('[ROS2BridgeClient] Connected to %s:%d\n', obj.host, obj.port);
            catch ME
                error('Failed to connect to bridge at %s:%d\nError: %s', ...
                    obj.host, obj.port, ME.message);
            end
        end
        
        function data = receive_message(obj, timeout)
            % Receive JSON message from TCP bridge
            % Returns parsed JSON as struct
            % Timeout in seconds (default: 5)
            
            if nargin < 2, timeout = 5; end
            if ~obj.isConnected
                error('Not connected to bridge. Call connect() first.');
            end
            
            try
                % Set timeout
                obj.tcpClient.Timeout = timeout;
                
                % Read all available data (read() with no count argument reads all available)
                tic;  % Start timer
                allBytes = [];
                
                while toc < timeout
                    % Try to read available bytes
                    try
                        availableBytes = obj.tcpClient.read(obj.tcpClient.BytesAvailable, 'uint8');
                        if ~isempty(availableBytes)
                            allBytes = [allBytes; availableBytes];
                        end
                    catch
                        % No data available yet
                    end
                    
                    % If we have at least 4 bytes, try to parse length
                    if length(allBytes) >= 4
                        break;
                    end
                    pause(0.01);  % Small delay to allow data to arrive
                end
                
                if length(allBytes) < 4
                    error('Failed to read message length (got %d bytes)', length(allBytes));
                end
                
                % Extract length from first 4 bytes
                msgLength = uint32(allBytes(1)) * 256^3 + ...
                            uint32(allBytes(2)) * 256^2 + ...
                            uint32(allBytes(3)) * 256 + ...
                            uint32(allBytes(4));
                
                % Sanity check
                if msgLength > 1048576
                    error('Invalid message length: %d bytes', msgLength);
                end
                
                % Read until we have length + 4 (for the header)
                totalNeeded = msgLength + 4;
                tic;
                while length(allBytes) < totalNeeded && toc < timeout
                    try
                        availableBytes = obj.tcpClient.read(obj.tcpClient.BytesAvailable, 'uint8');
                        if ~isempty(availableBytes)
                            allBytes = [allBytes; availableBytes];
                        end
                    catch
                        % No data available yet
                    end
                    pause(0.01);
                end
                
                if length(allBytes) < totalNeeded
                    error('Failed to read complete message (got %d of %d bytes)', ...
                        length(allBytes), totalNeeded);
                end
                
                % Extract message bytes (skip 4-byte header)
                msgBytes = allBytes(5:4+msgLength);
                
                % Convert bytes to string and parse JSON
                jsonStr = char(msgBytes);
                data = jsondecode(jsonStr);
                
            catch ME
                rethrow(ME);
            end
            end
        end
        
        function disconnect(obj)
            % Disconnect from TCP bridge
            if ~isempty(obj.tcpClient) && isvalid(obj.tcpClient)
                clear obj.tcpClient;
            end
            obj.isConnected = false;
            fprintf('[ROS2BridgeClient] Disconnected\n');
        end
        
        function status = is_connected(obj)
            % Check if currently connected
            status = obj.isConnected;
        end
    end
end
