#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple ROS2 controller bridge for quadrotor
Converts cmd_vel (Twist) messages into Gazebo model velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class QuadrotorController(Node):
    def __init__(self):
        super().__init__('quadrotor_controller')
        
        # Get robot_name from launch parameter
        self.declare_parameter('robot_name', 'quadrotor_1')
        self.robot_name = self.get_parameter('robot_name').value
        
        # Create subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publisher for odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            f'/{self.robot_name}/odom',
            10
        )
        
        # Create timer for publishing odometry (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_odom)
        
        # Store current velocity
        self.current_twist = Twist()
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0
        
        self.get_logger().info(f'Quadrotor controller initialized for {self.robot_name}')

    def cmd_vel_callback(self, msg: Twist):
        """Handle cmd_vel messages"""
        self.current_twist = msg
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.vyaw = msg.angular.z
        
        self.get_logger().debug(
            f'Received cmd_vel: linear=[{self.vx:.2f}, {self.vy:.2f}, {self.vz:.2f}] '
            f'angular=[{self.vyaw:.2f}]'
        )

    def publish_odom(self):
        """Publish odometry feedback"""
        # Simple integration for odometry
        dt = 0.1  # 10 Hz
        
        # Update position (simple integration)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        self.x += (self.vx * cos_yaw - self.vy * sin_yaw) * dt
        self.y += (self.vx * sin_yaw + self.vy * cos_yaw) * dt
        self.z += self.vz * dt
        self.yaw += self.vyaw * dt
        
        # Normalize yaw
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        
        # Orientation (yaw only, as quaternion)
        quat_z = math.sin(self.yaw / 2.0)
        quat_w = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.vyaw
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    
    controller = QuadrotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
