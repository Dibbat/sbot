#!/usr/bin/env python3
"""
ROS 2 TCP Bridge Server
Bidirectional communication between MATLAB and ROS2:
- Receives odometry/IMU data and forwards to TCP clients
- Receives velocity commands from TCP clients and publishes to ROS2
"""

import rclpy
from rclpy.node import Node
import json
import socket
import struct
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
import sys
import time
import os

class ROS2TCPBridge(Node):
    def __init__(self, port=9999):
        super().__init__('ros2_tcp_bridge')
        
        # Initialize attributes BEFORE creating subscriptions
        self.bridge_port = port
        self.client_list = []
        self.clients_lock = threading.Lock()
        self.is_running = True
        self.msg_stats = {'odom': 0, 'imu': 0, 'sent': 0, 'cmd_received': 0}
        
        self.get_logger().info(f'=== ROS 2 TCP Bridge ===')
        self.get_logger().info(f'Node: {self.get_namespace()}{self.get_name()}')
        self.get_logger().info(f'Domain ID: {os.environ.get("ROS_DOMAIN_ID", "0")}')
        
        # Subscribe to odometry and IMU topics (INCOMING from Gazebo)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/quadrotor_1/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info('Subscribed to /quadrotor_1/odom')
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/quadrotor_1/imu',
            self.imu_callback,
            10
        )
        self.get_logger().info('Subscribed to /quadrotor_1/imu')
        
        # Publisher for velocity commands (OUTGOING to drone controller)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/quadrotor_1/cmd_vel',
            10
        )
        self.get_logger().info('Created publisher for /quadrotor_1/cmd_vel')
        
        # Start TCP server in background thread
        self.server_thread = threading.Thread(target=self.tcp_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f'TCP Server listening on 0.0.0.0:{port}')
        self.get_logger().info('Ready to receive client connections')
    
    def tcp_server(self):
        """TCP Server accepting client connections"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_socket.bind(('0.0.0.0', self.bridge_port))
            server_socket.listen(5)
            server_socket.settimeout(1.0)
            
            self.get_logger().info(f'Server socket bound to 0.0.0.0:{self.bridge_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to bind socket: {e}')
            return
        
        try:
            while self.is_running:
                try:
                    client_socket, addr = server_socket.accept()
                    self.get_logger().info(f'Client connected: {addr}')
                    
                    with self.clients_lock:
                        self.client_list.append(client_socket)
                    
                    # Handle client in background thread
                    threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, addr),
                        daemon=True
                    ).start()
                    
                except socket.timeout:
                    continue
        finally:
            server_socket.close()
    
    def handle_client(self, client_socket, addr):
        """Handle individual client connection - receive velocity commands"""
        try:
            while self.is_running:
                try:
                    # Try to read 4-byte length prefix
                    client_socket.settimeout(1.0)
                    lengthBytes = client_socket.recv(4)
                    
                    if not lengthBytes:
                        break  # Client disconnected
                    
                    # Parse message length (big-endian)
                    msgLength = struct.unpack('>I', lengthBytes)[0]
                    
                    if msgLength > 1048576:  # Max 1MB
                        self.get_logger().warning(f'Invalid message length: {msgLength}')
                        break
                    
                    # Read message data
                    msgBytes = b''
                    while len(msgBytes) < msgLength:
                        chunk = client_socket.recv(min(4096, msgLength - len(msgBytes)))
                        if not chunk:
                            break
                        msgBytes += chunk
                    
                    if len(msgBytes) < msgLength:
                        break
                    
                    # Parse JSON command
                    jsonStr = msgBytes.decode('utf-8')
                    cmd_data = json.loads(jsonStr)
                    
                    # Handle velocity command
                    if cmd_data.get('type') == 'velocity_command':
                        self.publish_velocity_command(cmd_data)
                        self.msg_stats['cmd_received'] += 1
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().error(f'Client error: {e}')
                    break
                    
        finally:
            with self.clients_lock:
                if client_socket in self.client_list:
                    self.client_list.remove(client_socket)
            try:
                client_socket.close()
            except:
                pass
            self.get_logger().info(f'Client disconnected: {addr}')
    
    def publish_velocity_command(self, cmd_data):
        """Publish velocity command to drone"""
        try:
            vel = cmd_data.get('velocity', {})
            twist = Twist(
                linear=Vector3(
                    x=float(vel.get('linear_x', 0.0)),
                    y=float(vel.get('linear_y', 0.0)),
                    z=float(vel.get('linear_z', 0.0))
                ),
                angular=Vector3(
                    x=float(vel.get('angular_x', 0.0)),
                    y=float(vel.get('angular_y', 0.0)),
                    z=float(vel.get('angular_z', 0.0))
                )
            )
            self.cmd_vel_pub.publish(twist)
            if self.msg_stats['cmd_received'] % 10 == 0:
                self.get_logger().info(f'Velocity commands: {self.msg_stats["cmd_received"]} received and published')
        except Exception as e:
            self.get_logger().error(f'Error publishing velocity command: {e}')
    
    def send_to_clients(self, data_dict):
        """Send JSON data to all connected clients"""
        try:
            json_str = json.dumps(data_dict)
            json_bytes = json_str.encode('utf-8')
            
            # 4-byte big-endian length prefix
            length = struct.pack('>I', len(json_bytes))
            message = length + json_bytes
            
            with self.clients_lock:
                dead_clients = []
                for client in self.client_list:
                    try:
                        client.sendall(message)
                        self.msg_stats['sent'] += 1
                    except Exception as e:
                        dead_clients.append(client)
                
                # Remove dead clients
                for client in dead_clients:
                    self.client_list.remove(client)
                    
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
    
    def odom_callback(self, msg):
        """Receive odometry and forward to clients"""
        self.msg_stats['odom'] += 1
        data = {
            'type': 'odometry',
            'position': {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'z': float(msg.pose.pose.position.z)
            },
            'velocity': {
                'linear_x': float(msg.twist.twist.linear.x),
                'linear_y': float(msg.twist.twist.linear.y),
                'linear_z': float(msg.twist.twist.linear.z),
                'angular_x': float(msg.twist.twist.angular.x),
                'angular_y': float(msg.twist.twist.angular.y),
                'angular_z': float(msg.twist.twist.angular.z)
            }
        }
        if self.msg_stats['odom'] % 10 == 0:  # Log every 10 messages
            self.get_logger().info(f'Odom: {self.msg_stats["odom"]} received, {self.msg_stats["sent"]} sent to clients')
        self.send_to_clients(data)
    
    def imu_callback(self, msg):
        """Receive IMU and forward to clients"""
        self.msg_stats['imu'] += 1
        data = {
            'type': 'imu',
            'orientation': {
                'x': float(msg.orientation.x),
                'y': float(msg.orientation.y),
                'z': float(msg.orientation.z),
                'w': float(msg.orientation.w)
            },
            'angular_velocity': {
                'x': float(msg.angular_velocity.x),
                'y': float(msg.angular_velocity.y),
                'z': float(msg.angular_velocity.z)
            },
            'linear_acceleration': {
                'x': float(msg.linear_acceleration.x),
                'y': float(msg.linear_acceleration.y),
                'z': float(msg.linear_acceleration.z)
            }
        }
        self.send_to_clients(data)


import os

def main():
    rclpy.init()
    
    port = 9999
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f'Invalid port: {sys.argv[1]}, using default 9999')
    
    try:
        bridge = ROS2TCPBridge(port)
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print('\nShutting down...')
        if 'bridge' in locals():
            bridge.is_running = False
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'bridge' in locals():
                bridge.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
