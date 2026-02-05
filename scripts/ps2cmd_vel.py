#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy


class PSForward(Node):
    def __init__(self, quadrotor_id):
        super().__init__('ps_forward_' + quadrotor_id)
        
        self.scaling = 10.0
        self.quadrotor_id = quadrotor_id
        
        # Create subscriber and publisher
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/quadrotor_' + quadrotor_id + '/cmd_vel',
            10
        )
        
        self.get_logger().info(f'PS forward node initialized for quadrotor_{quadrotor_id}')

    def joy_callback(self, joy_msg):
        """Convert joystick input to velocity command"""
        ps_axes = joy_msg.axes
        
        # Create twist message from joystick axes
        command = Twist(
            linear=Vector3(
                x=ps_axes[3] * self.scaling,  # Right stick vertical
                y=ps_axes[2] * self.scaling,  # Right stick horizontal
                z=ps_axes[1] * self.scaling   # Left stick vertical
            ),
            angular=Vector3(
                x=0.0,
                y=0.0,
                z=ps_axes[0] * self.scaling   # Left stick horizontal (yaw)
            )
        )
        
        self.cmd_vel_publisher.publish(command)


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ps2cmd_vel.py <quadrotor_id>")
        sys.exit(1)
    
    rclpy.init(args=args)
    quadrotor_id = sys.argv[1]
    
    ps_forward = PSForward(quadrotor_id)
    
    try:
        rclpy.spin(ps_forward)
    except KeyboardInterrupt:
        pass
    finally:
        ps_forward.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
