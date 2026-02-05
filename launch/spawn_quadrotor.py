#!/usr/bin/env python3
"""
ROS2 launch file for spawning the quadrotor in Gazebo
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    imav_pkg = get_package_share_directory('imav_2017')
    
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([imav_pkg, 'urdf', 'quadrotor.urdf.xacro']),
        description='Path to the robot URDF or XACRO file'
    )
    
    id_arg = DeclareLaunchArgument('id', default_value='0', description='Quadrotor ID')
    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.5', description='Z position')
    R_arg = DeclareLaunchArgument('R', default_value='0', description='Roll orientation')
    P_arg = DeclareLaunchArgument('P', default_value='0', description='Pitch orientation')
    Y_arg = DeclareLaunchArgument('Y', default_value='0', description='Yaw orientation')
    
    # Load robot description from xacro
    robot_description_cmd = Command([
        'xacro', ' ',
        LaunchConfiguration('model'), ' ',
        'id:=', LaunchConfiguration('id')
    ])
    
    # Spawn robot node
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', ['quadrotor_', LaunchConfiguration('id')],
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('R'),
            '-P', LaunchConfiguration('P'),
            '-Y', LaunchConfiguration('Y'),
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_cmd, value_type=str)
        }],
        output='screen'
    )
    
    # Quadrotor controller (cmd_vel → odom bridge)
    controller_node = Node(
        package='imav_2017',
        executable='quadrotor_controller',
        parameters=[{
            'robot_name': ['quadrotor_', LaunchConfiguration('id')]
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        id_arg,
        x_arg,
        y_arg,
        z_arg,
        R_arg,
        P_arg,
        Y_arg,
        robot_state_publisher_node,
        spawn_robot_node,
        controller_node,
    ])
