#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

"""Simplified launch for Antobot Ant robot in Gazebo."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    
    # Pre-process the xacro outside of the launch file
    # You'll need to run: xacro src/antobot_sim_description/urdf/ant_v4_classic.urdf.xacro > /tmp/ant_classic.urdf
    # before launching this file
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': open('/tmp/ant_classic.urdf', 'r').read(),
             'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', '/tmp/ant_classic.urdf',
            '-entity', 'antobot_ant',
            '-z', '0.15'
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