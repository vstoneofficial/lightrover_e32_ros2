import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
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
    ])
