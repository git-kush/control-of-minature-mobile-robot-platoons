#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    # Get paths
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Get the path to the world file
    world_file = os.path.join(turtlebot3_dir, 'worlds', 'empty_world.world')
    
    # Get the path to the SDF model file
    turtlebot3_model_dir = os.path.join(turtlebot3_dir, 'models', 'turtlebot3_burger')
    turtlebot3_model_file = os.path.join(turtlebot3_model_dir, 'model.sdf')
    
    # Add a debug log to print the path
    ld.add_action(LogInfo(msg=f"Model file path: {turtlebot3_model_file}"))
    
    # Start Gazebo server with reset flag
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--verbose -r'  # Add reset flag to start fresh
        }.items(),
    )
    
    # Start Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        ),
    )
    
    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    # Define a consistent orientation for all robots (0.0 radians = facing forward)
    orientation = '0.0'
    
    # Spawn the robots one at a time with positions that are 1 meter apart
    for i in range(3):
        # Define position
        x_pos = float(i)
        y_pos = 0.0
        
        # Define robot name
        robot_name = f"turtlebot{i}"
        robot_ns = f"/tb{i}"
        
        # Add a debug log for this robot
        ld.add_action(LogInfo(msg=f"Spawning robot {robot_name} at x={x_pos}, y={y_pos}, orientation={orientation}"))
        
        # Spawn the robot with consistent orientation
        spawn_robot_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot_name}',
            arguments=[
                '-entity', robot_name,
                '-file', turtlebot3_model_file,
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', '0.01',
                '-Y', orientation,  # Set the same orientation for all robots
                '-robot_namespace', robot_ns
            ],
            output='screen'
        )
        
        ld.add_action(spawn_robot_cmd)
    
    return ld