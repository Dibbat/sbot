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
                
                % Read 4-byte length prefix (big-endian uint32)
                lengthBytes = read(obj.tcpClient, 4, 'uint8');
                
                if isempty(lengthBytes) || length(lengthBytes) < 4
                    error('Failed to read message length');
                end
                
                % Convert 4 bytes to uint32 (big-endian)
                msgLength = uint32(lengthBytes(1)) * 256^3 + ...
                            uint32(lengthBytes(2)) * 256^2 + ...
                            uint32(lengthBytes(3)) * 256 + ...
                            uint32(lengthBytes(4));
                
                % Sanity check - message shouldn't be > 1MB
                if msgLength > 1048576
                    error('Invalid message length: %d bytes', msgLength);
                end
                
                % Read message data
                msgBytes = read(obj.tcpClient, msgLength, 'uint8');
                
                if isempty(msgBytes) || length(msgBytes) < msgLength
                    error('Failed to read complete message (got %d of %d bytes)', ...
                        length(msgBytes), msgLength);
                end
                
                % Convert bytes to string and parse JSON
                jsonStr = char(msgBytes);
                data = jsondecode(jsonStr);
                
            catch ME
                if strcmp(ME.identifier, 'instrument:tcpip:read:timedOut')
                    error('Timeout waiting for message from bridge');
                else
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
