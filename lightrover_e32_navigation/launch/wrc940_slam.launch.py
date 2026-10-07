import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam_rviz = LaunchConfiguration('use_slam_rviz')
    use_keyboard_teleop = LaunchConfiguration('use_keyboard_teleop')
    keyboard_terminal_prefix = LaunchConfiguration('keyboard_terminal_prefix')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rvizconfig')

    bringup_launch = os.path.join(
        get_package_share_directory('lightrover_e32_bringup'),
        'launch',
        'wrc940_bringup.launch.py',
    )
    keyboard_teleop_launch = os.path.join(
        get_package_share_directory('lightrover_e32_bringup'),
        'launch',
        'keyboard_teleop.launch.py',
    )
    default_slam_params = os.path.join(
        get_package_share_directory('lightrover_e32_navigation'),
        'config',
        'wrc940_mapper_params_online_async.yaml',
    )
    default_rviz_config = get_package_share_path('lightrover_e32_navigation') / 'rviz/slam.rviz'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'use_slam_rviz',
            default_value='true',
            description='Start RViz2 with the SLAM configuration.',
        ),
        DeclareLaunchArgument(
            'use_keyboard_teleop',
            default_value='false',
            description='Start keyboard teleop for manual control during SLAM.',
        ),
        DeclareLaunchArgument(
            'keyboard_terminal_prefix',
            default_value='xterm -fa Monospace -fs 12 -e',
            description='Terminal prefix used by teleop_twist_keyboard.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Path to the slam_toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=str(default_rviz_config),
            description='Path to the RViz2 configuration file.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'use_rviz': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(keyboard_teleop_launch),
            launch_arguments={
                'terminal_prefix': keyboard_terminal_prefix,
            }.items(),
            condition=IfCondition(use_keyboard_teleop),
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config],
            condition=IfCondition(use_slam_rviz),
        ),
    ])
