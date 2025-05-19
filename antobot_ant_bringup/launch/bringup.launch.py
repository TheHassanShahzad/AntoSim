from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package share directories
    description_share = FindPackageShare('antobot_ant_description').find('antobot_ant_description')
    bringup_share = FindPackageShare('antobot_ant_bringup').find('antobot_ant_bringup')
    slam_toolbox_share = FindPackageShare('slam_toolbox').find('slam_toolbox')

    # RViz config path
    rviz_config_file = os.path.join(bringup_share, 'config', 'display.rviz')

    # Launch gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch SLAM Toolbox (online async mode)
    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': 'true'}.items()
    # )

    # Optional GUI toggle for RViz (you can extend this later)
    gui_arg = DeclareLaunchArgument('gui', default_value='True')
    show_gui = LaunchConfiguration('gui')

    # RViz node
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    return LaunchDescription([
        gui_arg,
        gazebo_launch,
        # slam_launch,
        # rviz_node
    ])
