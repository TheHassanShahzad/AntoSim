#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

"""Launch Antobot Ant robot in an empty Gazebo world for simulation."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path variables
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_antobot_sim_description = get_package_share_directory('antobot_sim_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch gazebo without pausing
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'pause': 'false',
        }.items(),
    )
    
    # Get URDF via xacro
    urdf_file = os.path.join(pkg_antobot_sim_description, 'urdf', 'ant_v4.urdf.xacro')
    
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                urdf_file
            ]
        ),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'antobot_ant'
        ],
        output='screen'
    )
    
    # Launch nodes
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])