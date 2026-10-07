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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam_rviz = LaunchConfiguration('use_slam_rviz')
    use_keyboard_teleop = LaunchConfiguration('use_keyboard_teleop')
    keyboard_terminal_prefix = LaunchConfiguration('keyboard_terminal_prefix')
    slam_params_file = LaunchConfiguration('slam_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    rviz_config = LaunchConfiguration('rvizconfig')

    nav_share = get_package_share_directory('lightrover_e32_navigation')
    description_share = get_package_share_path('lightrover_e32_description')

    default_slam_params = os.path.join(
        nav_share,
        'config',
        'wrc940_mapper_params_online_async.yaml',
    )
    default_ekf_params = os.path.join(
        nav_share,
        'config',
        'wrc940_ekf_imu.yaml',
    )
    default_rviz_config = get_package_share_path('lightrover_e32_navigation') / 'rviz/slam.rviz'
    keyboard_teleop_launch = os.path.join(
        get_package_share_directory('lightrover_e32_bringup'),
        'launch',
        'keyboard_teleop.launch.py',
    )
    urdf_path = description_share / 'urdf/lightrover_e32_robot.urdf'
    robot_description = ParameterValue(urdf_path.read_text(), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_slam_rviz', default_value='true'),
        DeclareLaunchArgument('use_keyboard_teleop', default_value='false'),
        DeclareLaunchArgument('keyboard_terminal_prefix', default_value='xterm -fa Monospace -fs 12 -e'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf_params),
        DeclareLaunchArgument('rvizconfig', default_value=str(default_rviz_config)),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='lightrover_e32_bringup',
            executable='pub_odom',
            name='pub_odom',
            parameters=[{
                'odom_topic': '/wheel/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': False,
                'linear_scale': 1.0,
                'angular_scale': 1.0,
                'pose_covariance_x': 0.02,
                'pose_covariance_y': 0.02,
                'pose_covariance_yaw': 0.10,
                'twist_covariance_linear_x': 0.02,
                'twist_covariance_angular_z': 0.05,
            }],
        ),
        Node(
            package='lightrover_e32_bringup',
            executable='lidar_raw_to_scan',
            name='lidar_raw_to_scan',
            parameters=[{
                'input_topic_0': '/rover_lidar_raw_0',
                'input_topic_1': '/rover_lidar_raw_1',
                'output_topic': '/scan',
                'frame_id': 'laser_frame',
                'scan_points': 360,
                'chunk_points': 180,
                'scan_time': 0.125,
                'stamp_offset': -0.125,
                'reverse': True,
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_params_file,
                {'use_sim_time': use_sim_time},
            ],
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(keyboard_teleop_launch),
            launch_arguments={
                'terminal_prefix': keyboard_terminal_prefix,
            }.items(),
            condition=IfCondition(use_keyboard_teleop),
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
