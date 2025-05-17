#!/usr/bin/env python3

"""Launch Antobot Ant robot in Ignition Gazebo."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Directory paths
    pkg_antobot_sim_description = get_package_share_directory('antobot_sim_description')
    
    # Check if ros_ign_gazebo package exists (for Ignition Gazebo)
    try:
        pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
        ign_gazebo_available = True
    except:
        # Try with new naming convention in newer versions
        try:
            pkg_ros_ign_gazebo = get_package_share_directory('ros_gz_sim')
            ign_gazebo_available = True
        except:
            ign_gazebo_available = False
            print("WARNING: Neither ros_ign_gazebo nor ros_gz_sim package found. Make sure Ignition Gazebo is installed.")
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Process URDF
    urdf_file = os.path.join(pkg_antobot_sim_description, 'urdf', 'ignition_simple.urdf.xacro')
    
    # Pre-process the URDF file before launch
    os.system(f'xacro {urdf_file} > /tmp/ant_ignition_simple.urdf')
    
    # Read the processed URDF
    with open('/tmp/ant_ignition_simple.urdf', 'r') as file:
        robot_description = file.read()
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Check which package name to use based on availability
    if ign_gazebo_available:
        # Launch Ignition Gazebo
        if os.path.exists(os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch')):
            # Old package name (ros_ign_gazebo)
            ign_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
                ),
                launch_arguments={
                    'ign_args': '-r empty.sdf'
                }.items()
            )
            
            # Spawn the robot in Ignition Gazebo
            spawn_entity = Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=[
                    '-name', 'antobot_ant',
                    '-topic', 'robot_description',
                    '-z', '0.15'
                ],
                output='screen'
            )
            
            # Ignition to ROS bridge
            bridge = Node(
                package='ros_ign_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/model/antobot_ant/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                    '/model/antobot_ant/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
                ],
                output='screen'
            )
        else:
            # New package name (ros_gz_sim)
            ign_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args': '-r empty.sdf'
                }.items()
            )
            
            # Spawn the robot in Ignition Gazebo
            spawn_entity = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'antobot_ant',
                    '-topic', 'robot_description',
                    '-z', '0.15'
                ],
                output='screen'
            )
            
            # Ignition to ROS bridge
            bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/model/antobot_ant/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/antobot_ant/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
                ],
                output='screen'
            )
    else:
        # If Ignition Gazebo is not available, print an error message
        print("ERROR: Ignition Gazebo (ros_ign_gazebo or ros_gz_sim) is required but not found.")
        print("Please install Ignition Gazebo and its ROS 2 packages.")
        ign_gazebo = ExecuteProcess(cmd=['echo', 'ERROR: Ignition Gazebo is not installed. Please install it.'], output='screen')
        spawn_entity = ExecuteProcess(cmd=['echo', 'ERROR: Cannot spawn entity without Ignition Gazebo.'], output='screen')
        bridge = ExecuteProcess(cmd=['echo', 'ERROR: Cannot create bridge without Ignition Gazebo.'], output='screen')
    
    # Launch description
    ld = LaunchDescription()
    
    # Add launch options
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'))
    
    # Add actions to launch
    ld.add_action(ign_gazebo)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(bridge)
    
    return ld