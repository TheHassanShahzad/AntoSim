from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare('antobot_ant_description').find('antobot_ant_description')

    # allow selecting world on CLI
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'default.world'),
        description='Gazebo world file'
    )
    world = LaunchConfiguration('world')

    # 1) Gazebo (server + client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )

    # 2) Parse URDF from Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'antobot_ant.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # 3) Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': True}
        ]
    )

    # 4) Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 5) Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'antobot_ant',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # 6) Front-camera MJPEG bridge
    camera_bridge = Node(
        package='antobot_ant_description',
        executable='ros_camera_flask',
        name='ros_camera_flask',
        output='screen'
    )

    # 7) Joystick → cmd_vel bridge
    joy_ctrl = Node(
        package='antobot_ant_description',
        executable='mqtt_joystick_controller',
        name='mqtt_joystick_controller',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        camera_bridge,
        joy_ctrl,
    ])
