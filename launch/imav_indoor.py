#!/usr/bin/env python3
"""
ROS2 launch file for IMAV 2017 Virtual Challenge
Launches Gazebo with the indoor environment and spawns the quadrotor
"""

import os
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory (prefer source tree if present)
    imav_pkg = get_package_share_directory('imav_2017')
    source_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    source_worlds = os.path.join(source_root, 'worlds', 'xacro', 'imav_indoor.world.xacro')
    source_urdf = os.path.join(source_root, 'urdf', 'quadrotor.urdf.xacro')
    source_meshes = os.path.join(source_root, 'meshes')
    use_source = os.path.exists(source_worlds) and os.path.exists(source_urdf)
    
    # Generate world from xacro to avoid hardcoded paths
    xacro_world_path = source_worlds if use_source else os.path.join(imav_pkg, 'worlds', 'xacro', 'imav_indoor.world.xacro')
    with tempfile.NamedTemporaryFile(prefix='imav_indoor_', suffix='.world', delete=False) as tmp_world:
        generated_world_path = tmp_world.name
    subprocess.run(['xacro', xacro_world_path, '-o', generated_world_path], check=True)
    
    # Define launch arguments for world
    world_name_arg = DeclareLaunchArgument(
        'world',
        default_value=generated_world_path,
        description='Full path to the Gazebo world file'
    )
    
    # Define launch arguments for quadrotor spawning
    id_arg = DeclareLaunchArgument('id', default_value='0', description='Quadrotor ID')
    x_arg = DeclareLaunchArgument('x', default_value='-2.5', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='2.5', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.5', description='Z position')
    R_arg = DeclareLaunchArgument('R', default_value='0', description='Roll orientation')
    P_arg = DeclareLaunchArgument('P', default_value='0', description='Pitch orientation')
    Y_arg = DeclareLaunchArgument('Y', default_value='0', description='Yaw orientation')
    
    # Build environment with HOME and model paths set
    env = os.environ.copy()
    if 'HOME' not in env:
        env['HOME'] = os.path.expanduser('~')
    
    # Set Gazebo model path to find meshes
    gazebo_model_path = source_meshes if use_source else os.path.join(imav_pkg, 'meshes')
    if 'GAZEBO_MODEL_PATH' in env:
        env['GAZEBO_MODEL_PATH'] = f"{gazebo_model_path}:{env['GAZEBO_MODEL_PATH']}"
    else:
        env['GAZEBO_MODEL_PATH'] = gazebo_model_path
    
    # Set Gazebo resource path
    resource_root = source_root if use_source else imav_pkg
    if 'GAZEBO_RESOURCE_PATH' in env:
        env['GAZEBO_RESOURCE_PATH'] = f"{resource_root}:{env['GAZEBO_RESOURCE_PATH']}"
    else:
        env['GAZEBO_RESOURCE_PATH'] = resource_root
    
    # Launch Gazebo server (headless - more stable for WSL)
    gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', LaunchConfiguration('world'), '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        env=env
    )
    
    # Load robot description from xacro
    xacro_file = source_urdf if use_source else os.path.join(imav_pkg, 'urdf', 'quadrotor.urdf.xacro')
    robot_description_cmd = Command([
        'xacro', ' ',
        xacro_file, ' ',
        'id:=', LaunchConfiguration('id')
    ])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_cmd, value_type=str)
        }],
        output='screen'
    )
    
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
    
    # Delay spawning
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot_node]
    )
    
    return LaunchDescription([
        world_name_arg,
        id_arg,
        x_arg,
        y_arg,
        z_arg,
        R_arg,
        P_arg,
        Y_arg,
        gazebo_server_cmd,
        robot_state_publisher_node,
        delayed_spawn,
    ])
