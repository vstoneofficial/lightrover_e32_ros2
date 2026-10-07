import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def expand_map_path(context):
    map_path = LaunchConfiguration('map').perform(context)
    return [SetLaunchConfiguration('expanded_map', os.path.expanduser(map_path))]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_localization_rviz = LaunchConfiguration('use_localization_rviz')
    map_file = LaunchConfiguration('expanded_map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rvizconfig')

    bringup_launch = os.path.join(
        get_package_share_directory('lightrover_e32_bringup'),
        'launch',
        'wrc940_bringup.launch.py',
    )
    localization_launch = os.path.join(
        get_package_share_directory('lightrover_e32_navigation'),
        'launch',
        'internal',
        'localization_stack.launch.py',
    )
    nav2_launch = os.path.join(
        get_package_share_directory('lightrover_e32_navigation'),
        'launch',
        'internal',
        'nav2_stack.launch.py',
    )
    default_params = os.path.join(
        get_package_share_directory('lightrover_e32_navigation'),
        'config',
        'wrc940_nav2_params.yaml',
    )
    default_map = os.path.join(
        get_package_share_directory('lightrover_e32_navigation'),
        'maps',
        'test.yaml',
    )
    default_rviz_config = get_package_share_path('lightrover_e32_navigation') / 'rviz/nav2.rviz'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_localization_rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rvizconfig', default_value=str(default_rviz_config)),
        OpaqueFunction(function=expand_map_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'namespace': '',
                'map': map_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': params_file,
                'use_composition': 'False',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
                'log_level': 'info',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'namespace': '',
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': params_file,
                'use_composition': 'False',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
                'log_level': 'info',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config],
            condition=IfCondition(use_localization_rviz),
        ),
    ])
