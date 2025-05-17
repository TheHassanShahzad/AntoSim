#!/usr/bin/env python3

"""Launch simplified Antobot Ant robot in Gazebo Classic for simulation."""

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
    
    # Get the URDF file path
    urdf_file = os.path.join(pkg_antobot_sim_description, 'urdf', 'gazebo_classic.urdf.xacro')
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'pause': 'false',
        }.items(),
    )
    
    # Process the URDF file before the launch
    os.system(f'xacro {urdf_file} > /tmp/ant_simple.urdf')
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': open('/tmp/ant_simple.urdf', 'r').read(),
             'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', '/tmp/ant_simple.urdf',
            '-entity', 'antobot_ant',
            '-z', '0.15'
        ],
        output='screen'
    )
    
    # Launch Description
    ld = LaunchDescription()
    
    # Add launch options
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'))
    
    # Add actions to launch
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)
    
    return ld