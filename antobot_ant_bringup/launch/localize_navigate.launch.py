import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('antobot_ant_bringup')
    description_dir = get_package_share_directory('antobot_ant_description')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # File paths
    bringup_launch = os.path.join(bringup_dir, 'launch', 'bringup.launch.py')
    map_file = PathJoinSubstitution([bringup_dir, 'maps', 'agriculture.yaml'])
    rviz_config = os.path.join(bringup_dir, 'config', 'localize_navigation.rviz')

    return LaunchDescription([
        # Launch robot simulation (bringup.launch.py should handle agriculture.world internally)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),


        # Explicit map_server (still useful for standalone debugging)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_server',
                '--ros-args',
                '-p', ['yaml_filename:=', map_file],
                '-p', 'use_sim_time:=true'
            ],
            output='screen'
        ),

        # Activate map_server lifecycle
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
            output='screen'
        ),

        # Run AMCL
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_amcl', 'amcl',
                '--ros-args',
                '-p', 'use_sim_time:=true'
            ],
            output='screen'
        ),

        # Activate AMCL lifecycle
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
            output='screen'
        ),

        # RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                'use_sim_time:=true',
                f'map:={map_file}'
            ],
            output='screen'
        )

    ])
