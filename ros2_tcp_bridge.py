#!/usr/bin/env python3
"""
ROS 2 TCP Bridge Server
Subscribes to ROS 2 topics and forwards data via TCP to Windows MATLAB
"""

import rclpy
from rclpy.node import Node
import json
import socket
import struct
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        self.msg_stats = {'odom': 0, 'imu': 0, 'sent': 0}
        
        self.get_logger().info(f'=== ROS 2 TCP Bridge ===')
        self.get_logger().info(f'Node: {self.get_namespace()}{self.get_name()}')
        self.get_logger().info(f'Domain ID: {os.environ.get("ROS_DOMAIN_ID", "0")}')
        
        # Subscribe to odometry and IMU topics
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
        """Handle individual client connection"""
        try:
            while self.is_running:
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'Client error: {e}')
        finally:
            with self.clients_lock:
                if client_socket in self.client_list:
                    self.client_list.remove(client_socket)
            try:
                client_socket.close()
            except:
                pass
            self.get_logger().info(f'Client disconnected: {addr}')
    
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
